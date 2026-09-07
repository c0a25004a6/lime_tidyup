# Cube8 projective geometry checkpoint — 2026-09-07

Minimal checkpoint for the current view-local Cube8 geometry result. The source
photo and rendered overlay are intentionally not committed.

Existing implementation on this branch:

- `yolo_ws/src/barcode_detector/barcode_detector/cube8_projective_hidden.py`
- `yolo_ws/src/barcode_detector/barcode_detector/cube8_two_face_view_local.py`

Contract:

`7 observed outer corners -> 3 vanishing directions -> 1 dependent hidden corner`

The derived corner is never counted as an independent observation and this
checkpoint makes no PnP, metric-3D, grasp, or robot-authority claim.

Latest photo measurement in image pixels:

```text
k0 = (668.25, 228.00) observed
k1 = (450.00, 236.00) observed
k2 = (814.00, 244.00) observed
k3 = (579.00, 252.00) observed
k4 = (588.00, 563.00) observed
k5 = (452.00, 491.00) observed
k6 = (814.00, 495.00) observed
k7 = (668.656880, 452.338371) projective-derived
axis-line RMS = 0.000125 px
```

The RMS is only an internal projective-consistency residual for the selected
outer corners; it is not a ground-truth pose-error measurement. High residual,
intersection spread, or visible-edge crossing must remain fail-closed.
