"""ROS2 Cube8 perception node: YOLO bbox -> RTMPose 8 vertices -> optional PnP/grasp candidates.

This node never commands an arm or gripper. Grasp output is geometric evidence only.
"""
from __future__ import annotations
import hashlib
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
from rtmlib.tools.pose_estimation.rtmpose import RTMPose

from .cube8_geometry import CUBE_EDGES
from .cube8_rtmpose import infer_rtmpose_mmpose_compatible, ordered_cube_keypoints_from_rtmpose
from .cube8_pnp import CameraModel, Cube8PnpError, estimate_cube_pose
from .cube8_grasp import Cube8GraspError, rank_cube_grasp_candidates

class Cube8PoseNode(Node):
    def __init__(self):
        super().__init__("cube8_pose_node")
        self.declare_parameter("image_topic","/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic","/camera/camera/color/camera_info")
        self.declare_parameter("result_topic","/cube8_pose_result")
        self.declare_parameter("debug_image_topic","/cube8_pose_debug_image")
        self.declare_parameter("detector_model_path","/root/yolo_ws/runs/pose/train-2/weights/best.pt")
        self.declare_parameter("rtmpose_model_path","/root/yolo_ws/models/cube8-rtmpose-retained.onnx")
        self.declare_parameter("detector_confidence",0.25)
        self.declare_parameter("keypoint_confidence",0.50)
        self.declare_parameter("bbox_expansion",1.0)
        self.declare_parameter("cube_side_length_m",0.057)
        self.declare_parameter("enable_pnp",True)
        self.declare_parameter("enable_grasp_candidates",True)
        self.declare_parameter("max_reprojection_error_px",4.0)
        self.declare_parameter("table_z_m",-10.0)
        self.declare_parameter("publish_debug_image",True)
        self.declare_parameter("inference_interval_sec",1.0)

        self.bridge=CvBridge()
        detector_path=str(self.get_parameter("detector_model_path").value)
        pose_path=str(self.get_parameter("rtmpose_model_path").value)
        self.detector=YOLO(detector_path)
        self.rtmpose=RTMPose(pose_path,model_input_size=(192,256),backend="onnxruntime",device="cpu")
        self.detector_sha256=_sha256(detector_path)
        self.rtmpose_sha256=_sha256(pose_path)
        self.camera_info=None
        self.last_time=0.0

        self.publisher=self.create_publisher(String,str(self.get_parameter("result_topic").value),10)
        self.debug_publisher=None
        if bool(self.get_parameter("publish_debug_image").value):
            self.debug_publisher=self.create_publisher(Image,str(self.get_parameter("debug_image_topic").value),10)
        self.create_subscription(CameraInfo,str(self.get_parameter("camera_info_topic").value),self.camera_info_callback,10)
        self.create_subscription(Image,str(self.get_parameter("image_topic").value),self.image_callback,10)

        self.get_logger().info(f"Cube8 node detector={detector_path} sha256={self.detector_sha256}")
        self.get_logger().info(f"Cube8 node rtmpose={pose_path} sha256={self.rtmpose_sha256}")
        self.get_logger().warning("Cube8 retained RTMPose is diagnostic/unqualified; no robot execution authority is emitted")

    def camera_info_callback(self,msg):
        self.camera_info=msg

    def image_callback(self,msg):
        now=time.time()
        interval=float(self.get_parameter("inference_interval_sec").value)
        if now-self.last_time < interval: return
        self.last_time=now
        try:
            frame=self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
            result=self.detector.predict(
                source=frame,
                conf=float(self.get_parameter("detector_confidence").value),
                save=False,verbose=False)[0]
            detections=self._process(frame,msg,result)
            payload={
                "timestamp":now,
                "frame_id":msg.header.frame_id,
                "pipeline":"yolo_bbox+cube8_rtmpose",
                "detector_sha256":self.detector_sha256,
                "rtmpose_sha256":self.rtmpose_sha256,
                "retained_model_qualification":"do_not_promote",
                "physical_robot_authority":False,
                "detections":detections,
            }
            out=String(); out.data=json.dumps(payload,ensure_ascii=False)
            self.publisher.publish(out)
            if self.debug_publisher is not None:
                dbg=self._draw_debug(frame,detections)
                debug_msg=self.bridge.cv2_to_imgmsg(dbg,encoding="bgr8")
                debug_msg.header=msg.header
                self.debug_publisher.publish(debug_msg)
        except Exception as exc:
            self.get_logger().error(f"Cube8 pose pipeline failed: {type(exc).__name__}: {exc}")

    def _process(self,frame,image_msg,result):
        if result.boxes is None: return []
        boxes=result.boxes.xyxy.cpu().tolist()
        classes=result.boxes.cls.cpu().tolist()
        confs=result.boxes.conf.cpu().tolist()
        names=result.names
        order=sorted(range(len(boxes)),key=lambda i:float(confs[i]),reverse=True)
        detections=[]
        h,w=frame.shape[:2]
        expansion=float(self.get_parameter("bbox_expansion").value)
        for idx in order:
            class_id=int(classes[idx])
            class_name=str(names[class_id])
            if "cube" not in class_name.lower(): continue
            x1,y1,x2,y2=[float(v) for v in boxes[idx]]
            cx=(x1+x2)/2; cy=(y1+y2)/2
            bw=x2-x1; bh=y2-y1
            from .cube8_rtmpose import expand_detection_bbox
            bbox=expand_detection_bbox(center_x=cx,center_y=cy,size_x=bw,size_y=bh,expansion=expansion,image_width=w,image_height=h)
            kp,scores=infer_rtmpose_mmpose_compatible(self.rtmpose,frame,[bbox.as_list()])
            ordered=ordered_cube_keypoints_from_rtmpose(
                kp,scores,visible_confidence=float(self.get_parameter("keypoint_confidence").value))
            item={
                "class_id":class_id,"class_name":class_name,"detector_confidence":float(confs[idx]),
                "bbox_xyxy":bbox.as_list(),
                "keypoints":[{"index":i,"label":p.label,"xy":[p.x,p.y],"confidence":p.confidence,"visibility":p.visibility}
                             for i,p in enumerate(ordered)],
                "pnp":{"status":"disabled"},
                "grasp":{"status":"not_computed","execution_authorized":False},
            }
            pose=None
            if bool(self.get_parameter("enable_pnp").value):
                item["pnp"]={"status":"unavailable","reason":"CameraInfo unavailable"}
                ci=self.camera_info
                if ci is not None:
                    if image_msg.header.frame_id and ci.header.frame_id and image_msg.header.frame_id != ci.header.frame_id:
                        item["pnp"]={"status":"rejected","reason":"image/CameraInfo frame mismatch"}
                    else:
                        try:
                            pose=estimate_cube_pose(
                                ordered,CameraModel(tuple(ci.k),tuple(ci.d)),
                                side_length_m=float(self.get_parameter("cube_side_length_m").value),
                                minimum_keypoint_confidence=float(self.get_parameter("keypoint_confidence").value),
                                maximum_reprojection_error_px=float(self.get_parameter("max_reprojection_error_px").value))
                            item["pnp"]={"status":"accepted","frame_id":ci.header.frame_id or image_msg.header.frame_id,**pose.as_dict()}
                        except Cube8PnpError as exc:
                            item["pnp"]={"status":"rejected","reason":str(exc)}
            if pose is not None and bool(self.get_parameter("enable_grasp_candidates").value):
                try:
                    candidates=rank_cube_grasp_candidates(
                        translation_m=pose.translation_m,
                        rotation_xyzw=pose.orientation_xyzw,
                        side_length_m=pose.side_length_m,
                        table_z_m=float(self.get_parameter("table_z_m").value))
                    item["grasp"]={
                        "status":"candidate_only",
                        "frame_id":item["pnp"].get("frame_id",""),
                        "requires_transform_to_robot_base":True,
                        "execution_authorized":False,
                        "candidates":[c.as_dict() for c in candidates[:6]],
                    }
                except Cube8GraspError as exc:
                    item["grasp"]={"status":"rejected","reason":str(exc),"execution_authorized":False}
            detections.append(item)
        return detections

    def _draw_debug(self,frame,detections):
        import cv2
        out=frame.copy()
        for item in detections:
            x1,y1,x2,y2=[int(round(v)) for v in item["bbox_xyxy"]]
            cv2.rectangle(out,(x1,y1),(x2,y2),(255,0,255),2)
            cv2.putText(out,f'{item["class_name"]} {item["detector_confidence"]:.2f}',(x1,max(20,y1-8)),
                        cv2.FONT_HERSHEY_SIMPLEX,0.55,(255,0,255),2,cv2.LINE_AA)
            pts=[tuple(int(round(v)) for v in p["xy"]) for p in item["keypoints"]]
            for a,b in CUBE_EDGES:
                cv2.line(out,pts[a],pts[b],(0,230,255),2,cv2.LINE_AA)
            for p,pt in zip(item["keypoints"],pts):
                color=(70+20*(p["index"]%3),220-15*(p["index"]%4),80+20*(p["index"]%5))
                cv2.circle(out,pt,5,color,-1,cv2.LINE_AA)
                cv2.putText(out,f'k{p["index"]}:{p["confidence"]:.2f}',(pt[0]+6,pt[1]-6),
                            cv2.FONT_HERSHEY_SIMPLEX,0.38,color,1,cv2.LINE_AA)
            if item["pnp"].get("status")=="accepted":
                cv2.putText(out,f'PnP z={item["pnp"]["translation_m"][2]:.3f}m err={item["pnp"]["reprojection_error_px"]:.2f}px',
                            (x1,min(out.shape[0]-10,y2+20)),cv2.FONT_HERSHEY_SIMPLEX,0.45,(0,255,0),1,cv2.LINE_AA)
                ci=self.camera_info
                if ci is not None:
                    import numpy as np
                    length=float(self.get_parameter("cube_side_length_m").value)*0.8
                    axes=np.asarray([[0,0,0],[length,0,0],[0,length,0],[0,0,length]],dtype=np.float64)
                    K=np.asarray(ci.k,dtype=np.float64).reshape(3,3)
                    d=np.asarray(ci.d,dtype=np.float64)
                    rvec=np.asarray(item["pnp"]["rotation_vector"],dtype=np.float64).reshape(3,1)
                    tvec=np.asarray(item["pnp"]["translation_m"],dtype=np.float64).reshape(3,1)
                    proj,_=cv2.projectPoints(axes,rvec,tvec,K,d)
                    q=[tuple(int(round(v)) for v in p) for p in proj.reshape(-1,2)]
                    cv2.line(out,q[0],q[1],(0,0,255),3,cv2.LINE_AA)
                    cv2.line(out,q[0],q[2],(0,255,0),3,cv2.LINE_AA)
                    cv2.line(out,q[0],q[3],(255,0,0),3,cv2.LINE_AA)
                grasp=item.get("grasp",{})
                if grasp.get("status")=="candidate_only" and grasp.get("candidates"):
                    c=grasp["candidates"][0]
                    rpy=c["grasp_frame_rpy_rad"]
                    cv2.putText(out,f'grasp {c["face_id"]}/{c["closing_axis_id"]} rpy=({rpy[0]:+.2f},{rpy[1]:+.2f},{rpy[2]:+.2f})',
                                (x1,min(out.shape[0]-10,y2+40)),cv2.FONT_HERSHEY_SIMPLEX,0.42,(0,200,255),1,cv2.LINE_AA)
        return out

def _sha256(path):
    h=hashlib.sha256()
    with open(path,"rb") as f:
        for chunk in iter(lambda:f.read(1024*1024),b""): h.update(chunk)
    return h.hexdigest()

def main(args=None):
    rclpy.init(args=args)
    node=Cube8PoseNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
