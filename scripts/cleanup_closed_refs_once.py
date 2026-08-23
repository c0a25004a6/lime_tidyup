#!/usr/bin/env python3
import json
import os
import urllib.error
import urllib.parse
import urllib.request

REPOSITORY = os.environ.get("GITHUB_REPOSITORY", "")
TOKEN = os.environ.get("GITHUB_TOKEN", "")
PR_NUMBER = os.environ.get("PR_NUMBER", "")
EXPECTED_REPOSITORY = "nekomario28/lime_tidyup"
MAINTENANCE_BRANCH = "maintenance/cleanup-closed-refs-2026-08-23"
TARGETS = {
    "fix/nav2-runtime-dependency": "ee0866ce1d3a2360abbf507ad14357bc30807da7",
}

if REPOSITORY != EXPECTED_REPOSITORY:
    raise SystemExit(f"unexpected repository: {REPOSITORY!r}")
if not TOKEN or not PR_NUMBER:
    raise SystemExit("GITHUB_TOKEN and PR_NUMBER are required")

BASE = f"https://api.github.com/repos/{REPOSITORY}"
HEADERS = {
    "Accept": "application/vnd.github+json",
    "Authorization": f"Bearer {TOKEN}",
    "Content-Type": "application/json",
    "X-GitHub-Api-Version": "2022-11-28",
    "User-Agent": "lime-tidyup-closed-ref-cleanup",
}


def encoded_ref(kind: str, name: str) -> str:
    return "/".join(urllib.parse.quote(part, safe="") for part in f"{kind}/{name}".split("/"))


def request(path: str, method: str = "GET", body=None, ok=(200,)):
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(BASE + path, data=data, method=method, headers=HEADERS)
    try:
        with urllib.request.urlopen(req) as resp:
            payload = resp.read()
            if resp.status not in ok:
                raise RuntimeError(f"unexpected HTTP {resp.status}: {path}")
            return None if not payload else json.loads(payload.decode("utf-8"))
    except urllib.error.HTTPError as exc:
        if exc.code in ok:
            return None
        detail = exc.read().decode("utf-8", errors="replace")[:500]
        raise RuntimeError(f"HTTP {exc.code} {path}: {detail}") from exc


def get_head(branch: str):
    try:
        payload = request(f"/git/ref/{encoded_ref('heads', branch)}")
    except RuntimeError as exc:
        if "HTTP 404" in str(exc):
            return None
        raise
    return payload["object"]["sha"]

observed = {}
for branch, expected_sha in TARGETS.items():
    if branch in {"main", "arm", "dev", "voice", MAINTENANCE_BRANCH}:
        raise SystemExit(f"refusing protected branch: {branch}")
    actual = get_head(branch)
    observed[branch] = actual
    if actual is not None and actual != expected_sha:
        raise SystemExit(
            f"refusing cleanup because {branch} moved: expected {expected_sha}, found {actual}"
        )

print("CLOSED_REF_PREFLIGHT=PASS")
for branch, actual in observed.items():
    if actual is None:
        print(f"already absent: {branch}")
        continue
    request(f"/git/refs/{encoded_ref('heads', branch)}", method="DELETE", ok=(204,))
    print(f"deleted: {branch}")

request(f"/pulls/{urllib.parse.quote(PR_NUMBER, safe='')}", method="PATCH", body={"state": "closed"})
print(f"closed cleanup PR #{PR_NUMBER}")
request(f"/git/refs/{encoded_ref('heads', MAINTENANCE_BRANCH)}", method="DELETE", ok=(204, 404))
print(f"deleted maintenance branch: {MAINTENANCE_BRANCH}")
print("CLOSED_REF_CLEANUP=PASS")
