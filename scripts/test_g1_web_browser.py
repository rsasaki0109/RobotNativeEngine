#!/usr/bin/env python3
"""Check a built G1 playground using an installed browser (no downloads)."""

import argparse
import io
from pathlib import Path

from PIL import Image
from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", required=True)
    parser.add_argument("--browser", default="/usr/bin/chromium")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(
            executable_path=args.browser,
            headless=True,
            args=["--disable-dev-shm-usage", "--enable-unsafe-webgpu",
                  "--enable-unsafe-swiftshader", "--use-angle=swiftshader",
                  "--enable-features=Vulkan", "--use-vulkan=swiftshader"],
        )
        page = browser.new_page(viewport={"width": 1440, "height": 900})
        errors = []
        page.on("pageerror", lambda error: errors.append(str(error)))
        page.on("console", lambda message: errors.append(message.text)
                if any(marker in message.text for marker in (
                    "Error while parsing WGSL", "Invalid ShaderModule", "GPUValidationError"
                )) else None)
        page.goto(args.url)
        page.wait_for_function(
            "document.querySelector('#engine-status').textContent.startsWith('操作できます')",
            timeout=90000,
        )
        assert page.locator("[data-angle]").count() == 23
        page.wait_for_function("document.querySelector('#canvas').width === 1440")
        page.evaluate("() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))")
        # The middle of the viewport contains the floor and G1, away from the
        # UI panels. A white/flat canvas must not pass as a working viewer.
        pixels = Image.open(io.BytesIO(page.screenshot(path=str(args.output / "g1-startup.png")))).convert("RGB")
        middle = pixels.crop((450, 100, 1000, 800))
        assert len(middle.getcolors(middle.width * middle.height)) > 100, "empty robot view"
        position = page.locator("#engine-position")
        initial = position.inner_text()
        page.locator("#canvas").click(position={"x": 700, "y": 450})
        page.keyboard.down("w")
        page.wait_for_function(
            "document.querySelector('#engine-position').textContent !== 'X 0.00 m · Z 0.00 m'"
        )
        page.keyboard.up("w")
        page.wait_for_function("document.querySelector('#control-forward').value === '0'")
        # Let pending redraws settle; the next six redraws must stay stopped.
        page.evaluate("() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))")
        stopped = position.inner_text()
        page.evaluate("""() => new Promise(resolve => {
            let remaining = 6;
            function tick() { if (--remaining) requestAnimationFrame(tick); else resolve(); }
            requestAnimationFrame(tick);
        })""")
        assert position.inner_text() == stopped
        page.keyboard.down("w")
        page.evaluate("window.dispatchEvent(new Event('blur'))")
        assert page.locator("#control-forward").input_value() == "0"
        page.keyboard.up("w")
        forward = page.locator('[data-drive="KeyW"]')
        forward.hover()
        page.mouse.down()
        assert page.locator("#control-forward").input_value() == "1"
        forward.dispatch_event("pointercancel", {"pointerId": 1})
        assert page.locator("#control-forward").input_value() == "0"
        page.mouse.up()
        page.locator("#control-reset").click()
        page.wait_for_function(
            "document.querySelector('#engine-position').textContent === 'X 0.00 m · Z 0.00 m'"
        )
        assert position.inner_text() == initial
        page.locator("#controls summary").click()
        page.locator("#control-joint-3").evaluate("""input => {
            input.value = "0.8"; input.dispatchEvent(new Event("input", {bubbles: true}));
        }""")
        actual = page.locator("#control-joint-3-value").evaluate("element => element.value")
        assert actual == "0.80 rad", actual
        page.screenshot(path=str(args.output / "g1-desktop.png"))
        page.locator("#control-reset").click()
        assert page.locator("#control-joint-3").input_value() == "0"
        page.locator("#controls summary").click()
        page.set_viewport_size({"width": 390, "height": 844})
        page.wait_for_function("document.querySelector('#canvas').width === 390")
        assert page.locator("#canvas").evaluate("canvas => canvas.clientWidth") == 390
        assert page.locator("#canvas").evaluate("canvas => canvas.height") == 422
        assert not page.evaluate("document.documentElement.scrollWidth > innerWidth")
        mobile = Image.open(io.BytesIO(page.screenshot(path=str(args.output / "g1-mobile.png")))).convert("RGB")
        stage = mobile.crop((0, 60, 390, 390))
        assert len(stage.getcolors(stage.width * stage.height)) > 100, "empty mobile robot view"
        assert not errors, errors
        browser.close()
    print("G1 browser checks passed: WASM startup, keyboard drive/stop, focus loss, pointer cancellation, joints, reset and responsive layout")


if __name__ == "__main__":
    main()
