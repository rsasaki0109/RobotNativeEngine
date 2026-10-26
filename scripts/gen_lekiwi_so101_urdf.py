#!/usr/bin/env python3
"""Generate assets/robots/lekiwi_so101/lekiwi_so101.urdf from lekiwi_base + so101."""

from __future__ import annotations

import argparse
import pathlib
import re
import textwrap

ROOT = pathlib.Path(__file__).resolve().parents[1]
LEKIWI = ROOT / "assets" / "robots" / "lekiwi" / "lekiwi_base.urdf"
SO101 = ROOT / "assets" / "robots" / "so101" / "so101.urdf"
OUT = ROOT / "assets" / "robots" / "lekiwi_so101" / "lekiwi_so101.urdf"

# Upstream LeKiwi mounts the first arm servo on Base_08q with this offset. SO-101
# base_link replaces Base_08q + holder stack; the rotation aligns Z-up CAD to the mount.
SO101_MOUNT_ORIGIN = (
    'xyz="-0.02975 -0.04565199 0.0278" rpy="1.5707963267948966 0 0"'
)


def rewrite_lekiwi_meshes(block: str) -> str:
    return re.sub(
        r'filename="package://lekiwi/([^"]+)"',
        r'filename="../lekiwi/\1"',
        block,
    )


def strip_collisions(block: str) -> str:
    return re.sub(r"\s*<collision\b[^>]*>.*?</collision>\s*", "\n", block, flags=re.S)


def rewrite_so101_meshes(block: str) -> str:
    return re.sub(
        r'filename="package://so101/([^"]+)"',
        r'filename="../so101/\1"',
        block,
    )


def strip_inertial(block: str) -> str:
    return re.sub(r"\s*<inertial>.*?</inertial>\s*", "\n", block, flags=re.S)


def strip_transmission(block: str) -> str:
    return re.sub(r"\s*<transmission\b.*?</transmission>\s*", "\n", block, flags=re.S)


def extract_blocks(xml: str, tag: str) -> list[str]:
    pattern = rf"<{tag}\b[^>]*>.*?</{tag}>"
    return re.findall(pattern, xml, re.S)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workbench", action="store_true",
                        help="retain SO-101 collisions/inertials and align CAD frames in a separate preview asset")
    parser.add_argument("--grasp", action="store_true", help="workbench model with explicit fingertip contact pads")
    args = parser.parse_args()
    if args.grasp:
        args.workbench = True
    output = OUT.with_name("lekiwi_so101_workbench.urdf") if args.workbench else OUT
    if args.grasp:
        output = OUT.with_name("lekiwi_so101_grasp.urdf")
    lekiwi_xml = LEKIWI.read_text(encoding="utf-8")
    so101_xml = SO101.read_text(encoding="utf-8")
    so101_xml = strip_transmission(so101_xml)

    lekiwi_inner = re.search(r"<robot[^>]*>(.*)</robot>", lekiwi_xml, re.S)
    if not lekiwi_inner:
        raise SystemExit("lekiwi_base.urdf missing <robot>")
    lekiwi_body = rewrite_lekiwi_meshes(lekiwi_inner.group(1))

    so101_links = []
    for block in extract_blocks(so101_xml, "link"):
        cleaned = strip_transmission(rewrite_so101_meshes(block))
        if not args.workbench:
            cleaned = strip_inertial(cleaned)
        if not args.workbench:
            cleaned = strip_collisions(cleaned)
        cleaned = cleaned.replace('name="base_link"', 'name="so101_base_link"', 1)
        if args.grasp:
            pads = {
                "gripper_link": ("-0.0079 -0.000218121 -0.0981274", "0.02 0.012 0.02"),
                "moving_jaw_so101_v1_link": ("0 -0.072 0.019", "0.016 0.01 0.016"),
            }
            link_name = re.search(r'link name="([^"]+)"', cleaned).group(1)
            if link_name in pads:
                origin, size = pads[link_name]
                cleaned = strip_collisions(cleaned)
                pad = f'<collision><origin xyz="{origin}"/><geometry><box size="{size}"/></geometry></collision>'
                visual = f'<visual><origin xyz="{origin}"/><geometry><box size="{size}"/></geometry><material name="grip_pad"><color rgba="0.12 0.18 0.2 1"/></material></visual>'
                cleaned = cleaned.replace('</link>', pad + visual + '</link>')
        so101_links.append(cleaned)

    so101_joints = []
    for block in extract_blocks(so101_xml, "joint"):
        cleaned = rewrite_so101_meshes(block)
        cleaned = re.sub(r"\s*<mimic\b[^>]*/>\s*", "\n", cleaned)
        cleaned = cleaned.replace('link="base_link"', 'link="so101_base_link"')
        if args.grasp and 'name="gripper"' in cleaned:
            cleaned = cleaned.replace('effort="10" velocity="10"', 'effort="0.06" velocity="1"')
        so101_joints.append(cleaned)

    # Both source models use Z-up CAD coordinates. The workbench applies the
    # single Z-up -> Y-up conversion to the assembled robot at spawn time.
    mount_origin = (
        SO101_MOUNT_ORIGIN.replace('rpy="1.5707963267948966 0 0"', 'rpy="0 0 0"')
        if args.workbench else SO101_MOUNT_ORIGIN
    )
    mount_joint = textwrap.dedent(
        f"""\
    <!-- Rigid mount: arm_mount is upstream Base_08q pose on base_plate_layer2-v3. -->
    <joint name="so101_mount_joint" type="fixed">
        <origin {mount_origin}/>
        <parent link="arm_mount"/>
        <child link="so101_base_link"/>
    </joint>"""
    )

    parts = [
        "<?xml version='1.0' encoding='utf-8'?>",
        '<robot name="lekiwi_so101">',
        "    <!-- LeKiwi kiwi-drive base + vendored SO-101 arm (generated). -->",
        "    <!-- Spawn with initial_rotation_rpy like lekiwi_base (Z-up CAD → Y-up ground). -->",
        *(extract_blocks(so101_xml, "material") if args.workbench else []),
        lekiwi_body.strip(),
        *so101_links,
        mount_joint,
        *so101_joints,
        "</robot>",
    ]

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(parts) + "\n", encoding="utf-8")
    print(f"wrote {output} ({len(so101_links)} arm links, {len(so101_joints)} arm joints)")


if __name__ == "__main__":
    main()
