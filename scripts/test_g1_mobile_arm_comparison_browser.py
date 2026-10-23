#!/usr/bin/env python3
"""Exercise six arm methods inside delivery with captured conditions and replay."""
import argparse
from pathlib import Path
from playwright.sync_api import sync_playwright
from test_g1_mobile_wait_browser import download
from test_g1_mobile_timeline_browser import choose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as p:
        browser = p.chromium.launch(executable_path='/usr/bin/chromium', headless=True, args=[
            '--disable-dev-shm-usage', '--enable-unsafe-webgpu', '--enable-unsafe-swiftshader',
            '--use-angle=swiftshader', '--enable-features=Vulkan', '--use-vulkan=swiftshader'])
        page = browser.new_page(viewport={'width': 1280, 'height': 900}, has_touch=True)
        errors = []
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.on('console', lambda message: errors.append(message.text) if any(x in message.text for x in ['GPUValidationError', 'Invalid ShaderModule', 'Error while parsing WGSL']) else None)
        page.goto(args.url)
        page.wait_for_function("document.querySelector('#engine-status').textContent.startsWith('操作できます')", timeout=90000)
        # The gallery must route the new action to the real WASM arm comparison.
        page.locator('[data-demo-id="delivery-arm"]').focus()
        page.keyboard.press('Enter')
        assert page.locator('#mobile-intent').input_value() == '4'
        page.wait_for_function('''()=>{
            if (document.querySelectorAll('#mobile-records tr').length !== 1) return false;
            document.querySelector('#mobile-pause-button').click();return true;
        }''', timeout=120000)
        page.wait_for_function("document.querySelector('#mobile-pause-button').getAttribute('aria-pressed')==='true'")
        assert '完了 1/6試行' in page.locator('#mobile-summary').text_content()
        assert page.locator('#mobile-timeline button:not(:disabled)').count() == 0
        frozen = page.locator('#mobile-status').text_content()
        page.wait_for_timeout(1600)
        assert page.locator('#mobile-status').text_content() == frozen
        page.locator('#mobile-step-button').click()
        page.wait_for_function('(old)=>document.querySelector("#mobile-status").textContent!==old', arg=frozen)
        # Edits during the run must never leak into later queued trials.
        page.locator('#manipulation-panel').evaluate('e=>e.open=true')
        page.locator('#mobile-scenario').select_option('2')
        page.locator('#arm-planner').select_option('0')
        page.locator('#experiment-planner-select').select_option('0')
        page.locator('#experiment-seed-editor').fill('43')
        page.locator('#experiment-seed-editor').dispatch_event('change')
        page.locator('#mobile-recovery-toggle').check()
        page.locator('#mobile-pause-button').click()
        page.wait_for_function('document.querySelectorAll("#mobile-records tr").length===6', timeout=360000)
        assert '完了 6/6試行' in page.locator('#mobile-summary').text_content()
        raw, records = download(page, args.output/'arm-delivery.csv')
        assert len(records) == 6
        assert [(r['arm_planner'], r['arm_controller']) for r in records] == [(str(p), str(c)) for p in range(3) for c in range(2)]
        assert all(r['seed'] == '42' and r['scenario_id'] == '0' and r['nav_planner'] == '1' and r['nav_controller'] == '1' and r['recovery'] == 'false' for r in records)
        assert all(r['expected_trials'] == '6' and r['completed_trials'] == '6' and r['batch_state'] == 'completed' for r in records)
        assert [r['result'] for r in records] == ['腕が障害物に接触', '腕が障害物に接触', '成功', '腕が障害物に接触', '成功', '成功']
        assert page.locator('#mobile-replay-record option').count() == 6
        assert page.locator('#mobile-timeline .timeline-row').count() == 6
        for phase, color in enumerate(['rgb(85, 222, 245)', 'rgb(205, 155, 255)', 'rgb(114, 237, 177)', 'rgb(246, 227, 121)']):
            assert page.locator(f'.timeline-segment[data-phase="{phase}"]').first.evaluate('e=>getComputedStyle(e).backgroundColor') == color
        assert page.locator('.timeline-terminal[data-success="false"]').first.evaluate('e=>getComputedStyle(e).color') == 'rgb(242, 155, 193)'
        for index, record in enumerate(records):
            choose(page, page.locator(f'.timeline-terminal[data-mobile-record="{index}"]'), keyboard=index == 5)
            assert ('全4段階完了' if record['result'] == '成功' else record['result']) in page.locator('#mobile-status').text_content()
        assert page.locator('#mobile-export-data').text_content() == raw.decode('utf-8-sig')
        page.screenshot(path=str(args.output/'arm-delivery.png'))
        # Test the direct button on mobile with the edited conditions.
        page.set_viewport_size({'width': 390, 'height': 844})
        page.locator('#mobile-arm-compare-button').tap()
        page.wait_for_function('document.querySelectorAll("#mobile-records tr").length===6 && document.querySelector("#mobile-summary").textContent.includes("6/6") && document.querySelector("#mobile-records").textContent.includes("seed 43")', timeout=120000)
        _, failed = download(page, args.output/'blocked-approach.csv')
        assert all(r['seed'] == '43' and r['scenario_id'] == '2' and r['nav_planner'] == '0' and r['recovery'] == 'true' and r['completed_stages'] == '0' for r in failed)
        page.screenshot(path=str(args.output/'arm-delivery-mobile.png'))
        page.locator('#mobile-stop-button').tap()
        page.wait_for_function("document.body.getAttribute('data-experiment-active')==='false'")
        assert not errors, errors
        print('PASS: six arm methods, gallery/keyboard/mobile launch, pause/step, captured configuration, stage outcomes, CSV and all terminal replays', flush=True)
        browser.close()


if __name__ == '__main__':
    main()
