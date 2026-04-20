#!/usr/bin/env python3
"""
STEP-AP242 assembly placement extractor for the Decode robot CAD.

Walks the NEXT_ASSEMBLY_USAGE_OCCURRENCE (NAUO) tree, composes
ITEM_DEFINED_TRANSFORMATION matrices, and prints each named component's
position + rotation in the top-level assembly frame.

Why text-parse instead of cadquery / pythonocc-core?
  cadquery wheels lag Python releases; pythonocc-core is conda-only on
  macOS. STEP is ASCII, and assembly placements only need NAUO + IDT +
  AXIS2_PLACEMENT_3D — no B-rep meshing. This parser handles those three
  entity types and that's enough for the camera-mount question.

Usage:
  python3 tools/parse_step.py [path/to/file.step]

If no path given, defaults to ~/Downloads/Top Level (2).step.

Limitations:
  * Bounding boxes / convex hulls / footprint polygons require geometry
    processing (B-rep mesh) and are NOT extracted here.
  * Parts placed directly under the root via MAPPED_ITEM (rather than CDSR)
    will report identity; we don't follow that path. Sub-assemblies and
    parts under sub-assemblies work fine.
  * Single-instance assumption: when a child PD has multiple parent NAUOs
    (re-used component) we pick the first.
"""

import math
import re
import sys
from pathlib import Path

import numpy as np


DEFAULT_STEP_PATH = Path.home() / "Downloads" / "Top Level (2).step"

# Components to report on. Add more as needed.
TARGETS_BY_NAME_SUBSTRING = [
    "Cam Global Shutter",
    "Limelight 3A <1>",
    "Limelight mount",
    "Pinpoint Odometry",
    "Main shooter <1>",
    "Vector Intake <1>",
    "right pod <1>",
    "left pod <1>",
]


# ---------- entity tokenization ----------

def load_entities(step_path: Path) -> dict[int, str]:
    text = step_path.read_text()
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    m = re.search(r"DATA;(.*?)ENDSEC;", text, flags=re.DOTALL)
    assert m, "STEP file has no DATA section"
    data = m.group(1)
    pat = re.compile(r"^\s*#(\d+)\s*=\s*(.+)\s*$", re.DOTALL)
    out: dict[int, str] = {}
    for raw in data.split(";"):
        m2 = pat.match(raw)
        if m2:
            out[int(m2.group(1))] = " ".join(m2.group(2).split())
    return out


def split_args(body: str) -> list[str]:
    s = body.find("(")
    e = body.rfind(")")
    if s < 0 or e < 0:
        return []
    inner = body[s + 1:e]
    out: list[str] = []
    cur: list[str] = []
    depth = 0
    in_str = False
    for ch in inner:
        if ch == "'":
            in_str = not in_str
            cur.append(ch)
        elif in_str:
            cur.append(ch)
        elif ch == "(":
            depth += 1
            cur.append(ch)
        elif ch == ")":
            depth -= 1
            cur.append(ch)
        elif ch == "," and depth == 0:
            out.append("".join(cur).strip())
            cur = []
        else:
            cur.append(ch)
    if cur:
        out.append("".join(cur).strip())
    return out


def refs(s: str) -> list[int]:
    return [int(x) for x in re.findall(r"#(\d+)", s)]


def floats_in(s: str) -> list[float]:
    return [float(x) for x in re.findall(r"-?\d+\.\d*[Ee]?[-+]?\d*|-?\d+\.|-?\d+", s)]


# ---------- placement parsing ----------

def parse_axis2(ents: dict[int, str], eid: int) -> tuple[list[float], list[float], list[float]]:
    args = split_args(ents[eid])
    loc_eid = refs(args[1])[0] if refs(args[1]) else None
    axis_eid = refs(args[2])[0] if len(args) > 2 and refs(args[2]) else None
    ref_eid = refs(args[3])[0] if len(args) > 3 and refs(args[3]) else None

    def get_vec(eid: int) -> list[float]:
        a = split_args(ents[eid])
        return floats_in(a[1]) if len(a) >= 2 else [0.0, 0.0, 0.0]

    loc = get_vec(loc_eid) if loc_eid else [0.0, 0.0, 0.0]
    z = get_vec(axis_eid) if axis_eid else [0.0, 0.0, 1.0]
    x = get_vec(ref_eid) if ref_eid else [1.0, 0.0, 0.0]
    return loc, z, x


def axes_to_T(loc: list[float], z: list[float], xref: list[float]) -> np.ndarray:
    z_arr = np.array(z, dtype=float)
    z_arr = z_arr / (np.linalg.norm(z_arr) + 1e-30)
    xr = np.array(xref, dtype=float)
    x = xr - np.dot(xr, z_arr) * z_arr
    nx = np.linalg.norm(x)
    if nx < 1e-12:
        tmp = np.array([1.0, 0.0, 0.0]) if abs(z_arr[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        x = tmp - np.dot(tmp, z_arr) * z_arr
        nx = np.linalg.norm(x)
    x = x / nx
    y = np.cross(z_arr, x)
    R = np.column_stack([x, y, z_arr])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array(loc, dtype=float)
    return T


def build_indexes(ents: dict[int, str]) -> dict:
    """Index NAUOs and the lookup chains needed to resolve placements."""
    nauo_info: dict[int, dict] = {}
    child_to_nauos: dict[int, list[int]] = {}
    for eid, body in ents.items():
        if not body.startswith("NEXT_ASSEMBLY_USAGE_OCCURRENCE"):
            continue
        args = split_args(body)
        if len(args) < 5:
            continue
        nauo_id = re.match(r"'([^']*)'", args[0])
        name = re.match(r"'([^']*)'", args[1])
        parent_pd = refs(args[3])[0] if refs(args[3]) else None
        child_pd = refs(args[4])[0] if refs(args[4]) else None
        nauo_info[eid] = {
            "nauo_id": nauo_id.group(1) if nauo_id else "?",
            "name": name.group(1) if name else "?",
            "parent_pd": parent_pd,
            "child_pd": child_pd,
        }
        if child_pd is not None:
            child_to_nauos.setdefault(child_pd, []).append(eid)

    # NAUO -> PRODUCT_DEFINITION_SHAPE -> CONTEXT_DEPENDENT_SHAPE_REPRESENTATION
    pds_by_nauo: dict[int, int] = {}
    for eid, body in ents.items():
        if body.startswith("PRODUCT_DEFINITION_SHAPE"):
            for r in refs(body):
                if r in nauo_info:
                    pds_by_nauo[r] = eid

    cdsr_for_pds: dict[int, int] = {}
    cdsr_to_rep_rel: dict[int, int] = {}
    for eid, body in ents.items():
        if body.startswith("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION"):
            args = split_args(body)
            if len(args) >= 2:
                rr = refs(args[0])[0] if refs(args[0]) else None
                df = refs(args[1])[0] if refs(args[1]) else None
                if rr and df:
                    cdsr_for_pds[df] = eid
                    cdsr_to_rep_rel[eid] = rr

    return {
        "nauo_info": nauo_info,
        "child_to_nauos": child_to_nauos,
        "pds_by_nauo": pds_by_nauo,
        "cdsr_for_pds": cdsr_for_pds,
        "cdsr_to_rep_rel": cdsr_to_rep_rel,
    }


def get_nauo_T(ents: dict[int, str], idx: dict, nauo_eid: int) -> np.ndarray | None:
    """Returns the 4x4 transform that takes points from the child part frame
    into the parent assembly frame, for the given NAUO."""
    pds = idx["pds_by_nauo"].get(nauo_eid)
    if not pds:
        return None
    cdsr = idx["cdsr_for_pds"].get(pds)
    if not cdsr:
        return None
    rr = idx["cdsr_to_rep_rel"].get(cdsr)
    if not rr:
        return None
    rrwt = re.search(
        r"REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\s*\(([^)]*)\)",
        ents[rr],
    )
    if not rrwt:
        return None
    idts = refs(rrwt.group(1))
    if not idts:
        return None
    args = split_args(ents[idts[0]])
    src_eid = refs(args[2])[0]
    tgt_eid = refs(args[3])[0]
    sl, sz, sx = parse_axis2(ents, src_eid)
    tl, tz, tx = parse_axis2(ents, tgt_eid)
    Tsrc = axes_to_T(sl, sz, sx)
    Ttgt = axes_to_T(tl, tz, tx)
    return np.linalg.inv(Tsrc) @ Ttgt


def chain_to_root(idx: dict, start_nauo: int) -> list[int]:
    chain = [start_nauo]
    cur = idx["nauo_info"][start_nauo]["parent_pd"]
    visited = {start_nauo}
    while True:
        parents = idx["child_to_nauos"].get(cur, [])
        if not parents:
            return chain
        nxt = parents[0]
        if nxt in visited:
            return chain
        visited.add(nxt)
        chain.append(nxt)
        cur = idx["nauo_info"][nxt]["parent_pd"]


def composed_transform(ents: dict[int, str], idx: dict, start_nauo: int) -> tuple[np.ndarray, list[int]]:
    chain = chain_to_root(idx, start_nauo)
    T = np.eye(4)
    for n in reversed(chain):
        Tn = get_nauo_T(ents, idx, n)
        if Tn is None:
            print(f"  WARNING: missing transform for NAUO #{n} ({idx['nauo_info'][n]['name']})", file=sys.stderr)
            continue
        T = T @ Tn
    return T, chain


# ---------- reporting ----------

def euler_zyx_from_R(R: np.ndarray) -> tuple[float, float, float]:
    """(yaw, pitch, roll) in radians."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return yaw, pitch, roll


def main():
    step_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_STEP_PATH
    if not step_path.exists():
        print(f"ERROR: STEP file not found: {step_path}", file=sys.stderr)
        sys.exit(1)
    print(f"Loading {step_path} ({step_path.stat().st_size / 1e6:.1f} MB)...", file=sys.stderr)
    ents = load_entities(step_path)
    print(f"  parsed {len(ents)} entities", file=sys.stderr)
    idx = build_indexes(ents)
    print(f"  {len(idx['nauo_info'])} NAUOs, {len(idx['pds_by_nauo'])} placements indexed", file=sys.stderr)

    matches = []
    for eid, info in idx["nauo_info"].items():
        for sub in TARGETS_BY_NAME_SUBSTRING:
            if sub in info["name"]:
                matches.append((eid, info))
                break

    print(f"\nFound {len(matches)} target NAUOs.\n", file=sys.stderr)
    print("=" * 72)
    for eid, info in matches:
        T, chain = composed_transform(ents, idx, eid)
        pos = T[:3, 3]
        chain_names = [idx["nauo_info"][c]["name"] for c in chain]
        yaw, pitch, roll = euler_zyx_from_R(T[:3, :3])
        print(f"\n{info['nauo_id']:8s} {info['name']}")
        print(f"  position (m, Top Level frame): ({pos[0]:9.6f}, {pos[1]:9.6f}, {pos[2]:9.6f})")
        print(f"  Euler ZYX (yaw, pitch, roll) [deg]: "
              f"({math.degrees(yaw):7.2f}, {math.degrees(pitch):7.2f}, {math.degrees(roll):7.2f})")
        print(f"  rotation matrix (cols = local axes in Top Level frame):")
        for row in T[:3, :3]:
            print(f"    [{row[0]:+8.4f}, {row[1]:+8.4f}, {row[2]:+8.4f}]")
        print(f"  assembly chain: {' <- '.join(chain_names)}")


if __name__ == "__main__":
    main()
