#!/usr/bin/env python3
"""Exercise the real workbench UI against a running local native host.

Requires Playwright and an installed Chrome/Chromium. The server must use the
bundled assets/robots root (the default). No browser or robot downloads are performed.
"""

import argparse
import json
from pathlib import Path

from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", required=True)
    parser.add_argument("--browser", default="/usr/bin/google-chrome")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[1]
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(
            executable_path=args.browser, headless=True,
            args=["--disable-dev-shm-usage"],
        )
        page = browser.new_page(viewport={"width": 1440, "height": 1080})
        page.set_default_timeout(30000)
        errors = []
        page.on("pageerror", lambda error: errors.append(str(error)))
        def expression(source):
            return "async () => { const state = await (await fetch('/api/state')).json(); return (" + source + "); }"

        def wait_state(source, timeout_ms=30000):
            page.wait_for_function(expression(source), timeout=timeout_ms)

        def inspect_state(source):
            return page.evaluate(expression(source))

        page.goto(args.url)
        page.wait_for_function("document.querySelector('#status').textContent.startsWith('接続済み')")
        page.select_option("#preset", "so101")
        wait_state("state.joints.length === 6")
        assert page.locator("#joints input[type=range]").count() == 6
        assert inspect_state("state.camera.depth_m.some(Number.isFinite)")
        assert inspect_state("state.lidar.ranges_m.length") == 180
        page.screenshot(path=str(args.output / "workbench.png"), full_page=True)

        page.fill("#pose-name", "home")
        page.click("#save-pose")
        wait_state("Boolean(state.project.poses.home)")
        page.get_by_label("shoulder_pan 目標値", exact=True).fill("0.3")
        page.get_by_label("shoulder_pan 目標値", exact=True).press("Tab")
        wait_state("state.project.targets.shoulder_pan.position_rad === 0.3")
        page.click("#play")
        wait_state("Number(state.sim_time_ticks) > 500000000")
        page.click("#play")
        page.get_by_role("button", name="▶ 再生", exact=True).wait_for()
        assert inspect_state("Math.abs(state.joints.find(j => j.info.name === 'shoulder_pan').measured)") > 0.005
        page.click("#apply-pose")
        wait_state("state.project.targets.shoulder_pan.position_rad === 0")

        page.click("#add-object")
        wait_state("state.project.objects.some(o => o.name === 'box-1')")
        page.fill("#object-name", "edited-box")
        page.fill("#pos-z", "1.3")
        page.click("#apply-object")
        wait_state("state.project.objects.some(o => o.name === 'edited-box' && o.position_m[2] === 1.3)")
        assert inspect_state("state.sim_time_ticks") == "0"
        with page.expect_download() as download:
            page.click("#download-project")
        project_path = args.output / "project.json"
        download.value.save_as(str(project_path))
        saved = json.loads(project_path.read_text())
        assert "home" in saved["poses"]

        page.select_option("#preset", "slider")
        wait_state("state.joints.length === 1")
        page.locator("#project-file").set_input_files(str(project_path))
        wait_state("state.joints.length === 6 && Boolean(state.project.poses.home)")
        assert inspect_state("state.project") == saved
        before = inspect_state("state.state_hash")
        invalid = json.loads(project_path.read_text())
        invalid["objects"].append(invalid["objects"][0])
        invalid_path = args.output / "invalid-project.json"
        invalid_path.write_text(json.dumps(invalid))
        page.locator("#project-file").set_input_files(str(invalid_path))
        page.wait_for_function("document.querySelector('#status').classList.contains('error')")
        actual = page.request.get(args.url + "/api/state").json()
        assert actual["state_hash"] == before
        assert actual["project"] == saved

        page.locator("#robot-file").set_input_files(
            str(root / "crates/rne_urdf_import/tests/fixtures/prismatic_slider.urdf")
        )
        wait_state("state.joints.length === 1")
        page.get_by_label("slider_joint 目標値", exact=True).fill("0.08")
        page.get_by_label("slider_joint 目標値", exact=True).press("Tab")
        wait_state("state.project.targets.slider_joint.position_m === 0.08")
        page.click("#step")
        wait_state("state.sim_time_ticks === '4166667'")
        page.locator("#robot-file").set_input_files(
            str(root / "crates/rne_mjcf/tests/fixtures/two_link_arm.xml")
        )
        wait_state("state.joints.length === 2 && state.project.robot.format === 'mjcf'")
        assert inspect_state("state.sim_time_ticks") == "0"
        page.set_viewport_size({"width": 390, "height": 844})
        assert not page.evaluate("document.documentElement.scrollWidth > innerWidth")
        page.screenshot(path=str(args.output / "workbench-mobile.png"), full_page=True)
        page.set_viewport_size({"width": 1440, "height": 1080})
        page.select_option("#preset", "lekiwi_so101")
        wait_state("Boolean(state.project.planar_base) && state.joints.length === 6")
        page.locator("#drive-panel").wait_for(state="visible")
        page.fill("#drive-x", "0.05")
        page.fill("#drive-z", "0.02")
        page.fill("#drive-yaw", "0.1")
        page.click("#drive-apply")
        wait_state("state.base.command.vx_m_s === 0.05 && state.base.command.vz_m_s === 0.02")
        page.click("#step")
        wait_state("Number(state.sim_time_ticks) > 0 && state.base.pose_x_z_yaw[0] > 0")
        page.click("#drive-stop")
        wait_state("state.base.command.vx_m_s === 0 && state.base.command.yaw_rad_s === 0")
        pose = inspect_state("state.base.pose_x_z_yaw")
        ticks = inspect_state("state.sim_time_ticks")
        page.click("#step")
        wait_state(f"state.sim_time_ticks !== '{ticks}'")
        assert inspect_state("state.base.pose_x_z_yaw") == pose
        with page.expect_download() as download:
            page.click("#download-recording")
        recording_path = args.output / "lekiwi-recording.json"
        download.value.save_as(str(recording_path))
        expected_hash = inspect_state("state.state_hash")
        page.click("#reset")
        wait_state("state.sim_time_ticks === '0'")
        page.locator("#recording-file").set_input_files(str(recording_path))
        wait_state(f"state.state_hash === '{expected_hash}'")
        assert inspect_state("state.base.command.vx_m_s") == 0
        page.evaluate("document.querySelector('main > aside').scrollTop = 0")
        page.screenshot(path=str(args.output / "lekiwi-workbench.png"), full_page=True)
        page.click("#grasp-demo")
        wait_state("Boolean(state.grasp_report) && state.grasp_report.minimum_hold_lift_m > 0.05")
        page.wait_for_function("document.querySelector('#grasp-report').textContent.includes('20回確認')")
        page.select_option("#objects", "target-block")
        assert page.input_value("#object-body") == "dynamic"
        assert float(page.input_value("#object-mass")) == 0.02
        assert inspect_state("state.objects.find(o => o.name === 'target-block').robot_contacts.length") == 2
        with page.expect_download() as download:
            page.click("#download-recording")
        grasp_path = args.output / "grasp-recording.json"
        download.value.save_as(str(grasp_path))
        expected_hash = inspect_state("state.state_hash")
        page.click("#reset")
        wait_state("state.sim_time_ticks === '0'")
        page.locator("#recording-file").set_input_files(str(grasp_path))
        wait_state(f"state.state_hash === '{expected_hash}'")
        page.screenshot(path=str(args.output / "contact-grasp.png"), full_page=True)
        page.fill("#object-mass", "0.03")
        page.click("#apply-object")
        wait_state("state.project.objects.find(o => o.name === 'target-block').dynamics.mass_kg === 0.03 && state.sim_time_ticks === '0'")
        with page.expect_download() as download:
            page.click("#download-project")
        dynamic_path = args.output / "dynamic-project.json"
        download.value.save_as(str(dynamic_path))
        page.select_option("#preset", "slider")
        wait_state("state.joints.length === 1")
        page.locator("#project-file").set_input_files(str(dynamic_path))
        wait_state("state.project.schema_version === 3 && state.joints.length === 6")
        assert inspect_state("state.project.objects.find(o => o.name === 'target-block').dynamics.mass_kg") == 0.03
        page.select_option("#preset", "lekiwi_pick_place")
        wait_state("state.project.objects.some(o => o.name === 'place-table')")
        assert page.input_value("#preset") == "lekiwi_pick_place"
        page.click("#pick-place-demo")
        wait_state("Boolean(state.pick_place_report) && state.pick_place_report.placement_samples === 20", timeout_ms=120000)
        page.wait_for_function("document.querySelector('#pick-place-report').textContent.includes('台上安定 2.0秒')")
        result = inspect_state("state.pick_place_report")
        assert result["transport_contact_samples"] == 80
        assert abs(result["final_position_m"][2] - 0.154) < 0.01
        assert inspect_state("state.objects.find(o => o.name === 'target-block').robot_contacts.length") == 0
        assert page.input_value("#preset") == "lekiwi_pick_place"
        page.screenshot(path=str(args.output / "pick-place.png"), full_page=True)
        with page.expect_download() as download:
            page.click("#download-recording")
        place_path = args.output / "pick-place-recording.json"
        download.value.save_as(str(place_path))
        expected_hash = inspect_state("state.state_hash")
        page.click("#reset")
        wait_state("state.sim_time_ticks === '0' && state.pick_place_report === null")
        page.locator("#recording-file").set_input_files(str(place_path))
        wait_state(f"state.state_hash === '{expected_hash}'", timeout_ms=120000)
        assert inspect_state("state.objects.find(o => o.name === 'target-block').robot_contacts.length") == 0
        assert inspect_state("state.base.command.vz_m_s") == 0
        assert not errors, errors
        browser.close()
    print("workbench browser checks passed: SO-101 servos, poses, scene edits, export/reload, atomic rejection, URDF/MJCF upload, RGB/depth/LiDAR, responsive layout, LeKiwi drive/stop, contact grasp/replay and dynamic-object persistence and pick/place replay")


if __name__ == "__main__":
    main()
