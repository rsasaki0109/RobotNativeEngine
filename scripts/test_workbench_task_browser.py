#!/usr/bin/env python3
"""Check resumable pick/place controls against a running local workbench."""
import argparse
import json
from pathlib import Path
from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", required=True)
    parser.add_argument("--browser", default="/usr/bin/chromium")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--destination", type=float, nargs=2, metavar=("X", "Z"), help="edit destination tabletop center before running")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as p:
        browser = p.chromium.launch(executable_path=args.browser, headless=True, args=["--disable-dev-shm-usage"])
        page = browser.new_page(viewport={"width": 1440, "height": 1080})
        page.set_default_timeout(120000)
        errors = []
        page.on("pageerror", lambda error: errors.append(str(error)))

        def state():
            return page.request.get(args.url + "/api/state", timeout=120000).json()

        def ready_click(selector):
            with page.expect_response(lambda r: r.url.endswith("/api/command")) as response:
                page.click(selector)
            assert response.value.ok, response.value.text()
            return response.value.json()

        page.goto(args.url)
        page.wait_for_function("document.querySelector('#status').textContent.startsWith('接続済み')")
        with page.expect_response(lambda r: r.url.endswith("/api/command")):
            page.select_option("#preset", "lekiwi_pick_place")
        page.select_option("#objects", "place-table")
        if args.destination:
            page.fill("#pos-x", str(args.destination[0]))
            page.fill("#pos-z", str(args.destination[1]))
            ready_click("#apply-object")
        page.fill("#pos-y", "0.2")
        invalid = ready_click("#apply-object")
        assert invalid["transfer_preview"]["plan"] is None
        page.wait_for_function("document.querySelector('#task-start').disabled")
        rejected = page.request.post(args.url + "/api/command", data={"command": "task_start"})
        assert not rejected.ok
        assert state()["state_hash"] == invalid["state_hash"]
        page.fill("#pos-y", "0.1845")
        prepared = ready_click("#apply-object")
        total_batches = 250 + prepared["transfer_preview"]["plan"]["batches"]
        destination = prepared["transfer_preview"]["plan"]["destination_m"]
        page.wait_for_function("!document.querySelector('#task-start').disabled")
        current = ready_click("#task-start")
        assert current["task"]["phase"] == "open" and current["sim_time_ticks"] == "0"
        current = ready_click("#step")
        assert current["task"]["phase_batch"] == 1
        assert current["sim_time_ticks"] == "100000008"
        assert page.get_by_label("gripper 目標値", exact=True).is_disabled()
        rejected = page.request.post(args.url + "/api/command", data={"command": "step", "steps": 1})
        assert not rejected.ok
        assert state()["sim_time_ticks"] == current["sim_time_ticks"]
        current = ready_click("#task-next")
        assert current["task"]["phase"] == "roll"
        before = int(current["sim_time_ticks"])
        page.click("#play")
        page.wait_for_function("document.querySelector('#clock').textContent !== '2.000 s'")
        page.click("#play")
        # An in-flight 0.1 s request may finish; no later requests may advance.
        page.wait_for_timeout(1500)
        paused = state()
        assert int(paused["sim_time_ticks"]) > before
        page.wait_for_timeout(300)
        assert state()["sim_time_ticks"] == paused["sim_time_ticks"]
        page.reload()
        page.wait_for_function(f"document.querySelector('#task-progress').textContent.includes('{total_batches}')")
        assert state()["sim_time_ticks"] == paused["sim_time_ticks"]
        current = paused
        while current["task"]["status"] == "running":
            current = ready_click("#task-next")
        assert current["task"]["status"] == "completed", current["task"]
        assert current["task"]["completed_batches"] == total_batches
        assert current["pick_place_report"]["placement_samples"] == 20
        final = current["pick_place_report"]["final_position_m"]
        assert ((final[0] - destination[0]) ** 2 + (final[2] - destination[2]) ** 2) ** 0.5 < 0.015
        assert current["pick_place_report"]["transport_contact_samples"] == total_batches - 250
        page.screenshot(path=str(args.output / "task-completed.png"), full_page=True)
        with page.expect_download() as download:
            page.click("#download-recording")
        recording = args.output / "task-recording.json"
        download.value.save_as(str(recording))
        expected_hash = current["state_hash"]
        assert len(json.loads(recording.read_text())["frames"]) == total_batches
        with page.expect_response(lambda r: r.url.endswith("/api/command")) as response:
            page.locator("#recording-file").set_input_files(str(recording))
        replayed = response.value.json()
        assert response.value.ok and replayed["state_hash"] == expected_hash
        assert replayed["task"] is None
        # Restart restores neutral motor targets even after a completed run.
        current = ready_click("#task-start")
        assert all(target.get("position_rad", 0) == 0 for target in current["project"]["targets"].values())
        for _ in range(2):
            ready_click("#task-next")
        current = ready_click("#step")
        assert current["base"]["command"]["vx_m_s"] > 0
        cancelled = ready_click("#task-cancel")
        assert cancelled["task"] is None and cancelled["base"]["command"]["vx_m_s"] == 0
        assert cancelled["sim_time_ticks"] == current["sim_time_ticks"]
        assert not page.get_by_label("gripper 目標値", exact=True).is_disabled()
        # A changed scene remains visible at the failing phase.
        page.select_option("#objects", "target-block")
        page.fill("#pos-z", "-0.25")
        ready_click("#apply-object")
        ready_click("#task-start")
        for _ in range(4):
            current = ready_click("#task-next")
        assert current["task"]["status"] == "failed"
        assert current["task"]["phase"] == "close"
        assert "both grasp pads" in current["task"]["error"]
        assert page.locator("#step").is_disabled()
        assert page.locator("#task-next").is_disabled()
        page.screenshot(path=str(args.output / "task-failed.png"), full_page=True)
        ticks = current["sim_time_ticks"]
        rejected = page.request.post(args.url + "/api/command", data={"command": "task_advance", "to_boundary": False})
        assert not rejected.ok and state()["sim_time_ticks"] == ticks
        assert not errors, errors
        browser.close()
    print("task browser checks passed: pause/resume, reload, phase stepping, completion, replay, neutral restart, cancellation, manual-control guard, retained failure, edited destination and invalid-height rejection")


if __name__ == "__main__":
    main()
