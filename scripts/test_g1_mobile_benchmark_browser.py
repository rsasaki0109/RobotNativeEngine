#!/usr/bin/env python3
"""Verify a thirty-trial delivery benchmark, CSV-derived aggregates and replay."""
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
        # Lower raster resolution keeps software WebGPU practical; simulation ticks are unchanged.
        page = browser.new_page(viewport={'width': 1280, 'height': 900}, device_scale_factor=0.5, has_touch=True)
        errors = []
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.on('console', lambda message: errors.append(message.text) if any(x in message.text for x in ['GPUValidationError', 'Invalid ShaderModule', 'Error while parsing WGSL']) else None)
        page.goto(args.url)
        page.wait_for_function("document.querySelector('#engine-status').textContent.startsWith('操作できます')", timeout=90000)
        page.locator('#experiment-panel').evaluate('e=>e.open=true')
        page.locator('#mobile-panel').evaluate('e=>e.open=true')
        page.locator('#manipulation-panel').evaluate('e=>e.open=true')
        for selector, value in [('#experiment-planner-select', '1'), ('#experiment-controller-select', '1'), ('#mobile-scenario', '0')]:
            page.locator(selector).select_option(value)
        page.locator('#experiment-seed-editor').fill('42')
        page.locator('#experiment-seed-editor').dispatch_event('change')
        page.locator('#mobile-benchmark-button').focus()
        page.keyboard.press('Enter')
        assert page.locator('#mobile-intent').input_value() == '5'
        page.wait_for_function('''()=>{
            if (document.querySelectorAll('#mobile-records tr').length!==1) return false;
            document.querySelector('#mobile-pause-button').click();return true;
        }''', timeout=120000)
        page.wait_for_function("document.querySelector('#mobile-pause-button').getAttribute('aria-pressed')==='true'")
        assert '完了 1/30試行' in page.locator('#mobile-summary').text_content()
        assert page.locator('.mobile-benchmark-table button:not(:disabled)').count() == 0
        assert page.locator('.mobile-benchmark-table tbody tr').count() == 6
        _, partial = download(page, args.output/'partial.csv')
        assert len(partial) == 1 and partial[0]['expected_trials'] == '30' and partial[0]['completed_trials'] == '1'
        frozen = page.locator('#mobile-status').text_content()
        page.wait_for_timeout(1200)
        assert page.locator('#mobile-status').text_content() == frozen
        page.locator('#mobile-step-button').click()
        page.wait_for_function('(old)=>document.querySelector("#mobile-status").textContent!==old', arg=frozen)
        for selector, value in [('#experiment-planner-select', '0'), ('#mobile-scenario', '2'), ('#arm-planner', '0')]:
            page.locator(selector).select_option(value)
        page.locator('#experiment-seed-editor').fill('99')
        page.locator('#experiment-seed-editor').dispatch_event('change')
        page.locator('#mobile-recovery-toggle').check()
        print('PASS: partial thirty-trial counts, pause/step and disabled summary replay', flush=True)
        page.locator('#mobile-pause-button').click()
        page.wait_for_function('document.querySelectorAll("#mobile-records tr").length===30', timeout=600000)
        raw, records = download(page, args.output/'benchmark.csv')
        assert len(records) == 30
        assert [r['seed'] for r in records] == [str(42 + i // 6) for i in range(30)]
        assert [(r['arm_planner'], r['arm_controller']) for r in records] == [(str(i % 6 // 2), str(i % 2)) for i in range(30)]
        assert all(r['expected_trials'] == '30' and r['completed_trials'] == '30' and r['batch_state'] == 'completed' and r['scenario_id'] == '0' and r['nav_planner'] == '1' and r['nav_controller'] == '1' and r['recovery'] == 'false' for r in records)
        stages = ['棚へ移動・整列', '把持・持ち上げ', '配置先へ運搬・整列', '配置']
        rows = page.locator('.mobile-benchmark-table tbody tr')
        for method in range(6):
            trials = records[method::6]
            successes = [r for r in trials if r['result'] == '成功']
            cells = rows.nth(method).locator('td')
            assert cells.nth(1).text_content() == '5/5'
            assert cells.nth(2).text_content() == f'{len(successes)}/5'
            assert cells.nth(3).text_content() == f'{len(successes)*20}%'
            if successes:
                assert abs(float(cells.nth(4).text_content()) - sum(float(r['total_s']) for r in successes)/len(successes)) < 0.011
            else:
                assert cells.nth(4).text_content() == '—'
            for stage, label in enumerate(stages):
                failures = sum(r['result'] != '成功' and int(r['completed_stages']) == stage for r in trials)
                assert f'{label}: {failures}' in cells.nth(5).text_content()
            assert cells.nth(6).locator('button:enabled').count() == 5
        assert any(r['result'] == '成功' for r in records) and any(r['result'] != '成功' for r in records)
        assert page.locator('#mobile-replay-record option').count() == 30
        assert 'seed 46' in page.locator('#mobile-replay-record option').last.text_content()
        summary = page.locator('#mobile-summary').inner_html()
        chosen = [0, next(i for i, r in enumerate(records) if r['result'] == '成功'), 29]
        for index in chosen:
            choose(page, page.locator(f'.mobile-benchmark-table button[data-mobile-record="{index}"]'), keyboard=index == 29)
            assert ('全4段階完了' if records[index]['result'] == '成功' else records[index]['result']) in page.locator('#mobile-status').text_content()
        assert page.locator('#mobile-summary').inner_html() == summary
        assert page.locator('#mobile-export-data').text_content() == raw.decode('utf-8-sig')
        page.locator('.mobile-benchmark-table').scroll_into_view_if_needed()
        page.screenshot(path=str(args.output/'benchmark.png'))
        print('PASS: thirty captured trials, CSV-derived method rates/means/failure stages and summary replay', flush=True)
        # Start edited settings on mobile; blocked approaches quickly finish all thirty.
        page.set_viewport_size({'width': 390, 'height': 844})
        page.locator('#mobile-benchmark-button').tap()
        page.wait_for_function('document.querySelectorAll("#mobile-records tr").length===30 && document.querySelector("#mobile-records").textContent.includes("seed 99")', timeout=120000)
        _, blocked = download(page, args.output/'blocked.csv')
        assert all(r['nav_planner'] == '0' and r['recovery'] == 'true' and r['scenario_id'] == '2' and r['completed_stages'] == '0' for r in blocked)
        assert set(r['seed'] for r in blocked) == {'99', '100', '101', '102', '103'}
        page.screenshot(path=str(args.output/'benchmark-mobile.png'))
        page.locator('#mobile-stop-button').tap()
        page.wait_for_function("document.body.getAttribute('data-experiment-active')==='false'")
        assert not errors, errors
        print('PASS: mobile benchmark launch/stop and captured edited settings', flush=True)
        browser.close()


if __name__ == '__main__':
    main()
