#!/usr/bin/env python3
"""Exercise the real WASM navigation experiment UI with an installed Chromium."""
import argparse
from pathlib import Path
from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', required=True)
    parser.add_argument('--browser', default='/usr/bin/chromium')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as p:
        browser = p.chromium.launch(executable_path=args.browser, headless=True, args=[
            '--disable-dev-shm-usage', '--enable-unsafe-webgpu', '--enable-unsafe-swiftshader',
            '--use-angle=swiftshader', '--enable-features=Vulkan', '--use-vulkan=swiftshader'])
        page = browser.new_page(viewport={'width': 1280, 'height': 800})
        errors = []
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.on('console', lambda msg: errors.append(msg.text) if any(marker in msg.text for marker in
            ['GPUValidationError', 'Invalid ShaderModule', 'Error while parsing WGSL']) else None)
        page.goto(args.url)
        page.wait_for_function("document.querySelector('#engine-status').textContent.startsWith('操作できます')", timeout=90000)
        page.locator('#experiment-task-select').select_option('1')
        page.locator('#experiment-run-button').click()
        page.wait_for_function("document.querySelector('#experiment-status').textContent.includes('計画失敗')")
        assert page.locator('#experiment-records tr').count() == 1
        assert page.locator('#experiment-records td').nth(4).inner_text() == '—'
        page.locator('#experiment-planner-select').select_option('1')
        page.locator('#experiment-controller-select').select_option('1')
        page.locator('#experiment-run-button').click()
        page.wait_for_function("document.querySelector('#experiment-status').textContent.includes('A* × DWA · 実行中')")
        assert page.locator('#experiment-map rect').count() == 2
        assert page.locator('#playground').get_attribute('data-experiment-active') == 'true'
        page.locator('#experiment-pause-button').click()
        page.wait_for_function("document.querySelector('#experiment-pause-button').getAttribute('aria-pressed') === 'true'")
        paused = page.locator('#experiment-status').inner_text()
        position = page.locator('#engine-position').inner_text()
        page.evaluate('() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))')
        assert page.locator('#experiment-status').inner_text() == paused
        assert page.locator('#engine-position').inner_text() == position
        page.locator('#experiment-step-button').click()
        page.wait_for_function('(previous) => document.querySelector("#experiment-status").textContent !== previous', arg=paused)
        assert page.locator('#experiment-pause-button').get_attribute('aria-pressed') == 'true'
        page.screenshot(path=str(args.output / 'experiment-desktop.png'))
        print('Planning failure, A*/DWA, pause, step and rendered overlay passed', flush=True)
        # Invalid/intermediate number edits must not terminate the WASM loop.
        page.locator('#experiment-seed-editor').fill('')
        page.locator('#experiment-task-select').select_option('0')
        assert page.locator('#experiment-seed').input_value() == '42'
        page.locator('#experiment-compare-button').click()
        page.wait_for_function('document.querySelectorAll("#experiment-records tr").length === 4', timeout=240000)
        rows = page.locator('#experiment-records tr').all_inner_texts()
        assert all('成功' in row and 'seed 42' in row for row in rows), rows
        assert any('Pure Pursuit' in row for row in rows) and any('DWA' in row for row in rows)
        page.locator('#experiment-results-button').click()
        page.screenshot(path=str(args.output / 'experiment-results.png'))
        print('Four-pair comparison finished with genuine results', flush=True)
        # Completed traces persist together; legend toggles affect only drawing.
        assert page.locator('#experiment-map polyline').count() == 5
        assert '成功 4/4' in page.locator('#experiment-comparison').inner_text()
        page.locator('[data-trace="2"]').uncheck()
        page.wait_for_function('document.querySelectorAll("#experiment-map polyline").length === 4')
        assert page.locator('#experiment-records tr').all_inner_texts() == rows
        page.locator('[data-trace="2"]').check()
        page.wait_for_function('document.querySelectorAll("#experiment-map polyline").length === 5')
        detail = page.locator('#experiment-records details').first
        detail.locator('summary').click()
        page.evaluate('() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))')
        assert detail.get_attribute('open') is not None
        detail.locator('summary').click()
        for task, title in [('4', '狭い通路'), ('6', '行き止まりを迂回'), ('7', '3地点を巡回')]:
            page.locator('#experiment-task-select').select_option(task)
            assert page.locator('#experiment-task-description').inner_text()
            page.locator('#experiment-compare-button').click()
            page.wait_for_function('(title) => document.querySelectorAll("#experiment-records tr").length === 4 && [...document.querySelectorAll("#experiment-records tr")].every(r => r.textContent.includes(title))', arg=title, timeout=240000)
            current = page.locator('#experiment-records tr').all_inner_texts()
            if task == '6':
                assert any('計画失敗' in row for row in current), current
                assert any('接触で停止' in row for row in current), current
                assert any('成功' in row for row in current), current
            else:
                assert all('成功' in row for row in current), current
            assert page.locator('#experiment-map polyline').count() == 5
            if task == '7': assert all('到達 3地点' in row for row in page.locator('#experiment-records tr').all_text_contents()), current
            print(f'Comparison passed: {title}', flush=True)
        page.screenshot(path=str(args.output / 'patrol-comparison.png'))
        page.locator('#experiment-task-select').select_option('5')
        page.locator('#experiment-run-button').click()
        page.wait_for_function("document.querySelector('#experiment-status').textContent.includes('横切る障害物')")
        page.locator('#experiment-pause-button').click()
        page.wait_for_function("document.querySelector('#experiment-pause-button').getAttribute('aria-pressed') === 'true'")
        obstacle_y = page.locator('#experiment-map rect').nth(1).get_attribute('y')
        page.locator('#experiment-step-button').click()
        page.wait_for_function('(old) => document.querySelectorAll("#experiment-map rect")[1].getAttribute("y") !== old', arg=obstacle_y)
        assert page.locator('#experiment-map polyline').count() == 2  # no previous batch traces
        print('Task catalog, trace overlays/toggles, U-turn outcomes, patrol and moving obstacle passed', flush=True)
        page.locator('#experiment-manual-button').click()
        page.wait_for_function("document.querySelector('#playground').dataset.experimentActive === 'false'")
        page.locator('#control-reset').click()
        page.wait_for_function("document.querySelector('#engine-position').textContent === 'X 0.00 m · Z 0.00 m'")
        assert page.locator('#experiment-records tr').count() >= 4
        page.set_viewport_size({'width': 390, 'height': 844})
        page.wait_for_function("document.querySelector('#canvas').width === 390")
        assert page.locator('#controls').evaluate('e => getComputedStyle(e).visibility') == 'hidden'
        assert page.locator('#experiment-panel > summary').is_visible()
        assert not page.evaluate('document.documentElement.scrollWidth > innerWidth')
        page.locator('#experiment-panel').evaluate('e => { e.scrollTop = 0; }')
        page.screenshot(path=str(args.output / 'experiment-mobile.png'))
        page.locator('#experiment-panel > summary').click()
        assert page.locator('#controls').is_visible()
        assert not errors, errors
        browser.close()
    print('Experiment browser checks passed: selectors, real planner/controller runs, failure metrics, pause/step, comparison, manual/reset and mobile layout')


if __name__ == '__main__':
    main()
