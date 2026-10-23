#!/usr/bin/env python3
"""Verify paired recovery policies with fixed methods, separate statistics and replay."""
import argparse
from pathlib import Path
from playwright.sync_api import sync_playwright
from test_g1_mobile_browser import seek
from test_g1_mobile_wait_browser import download


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
        page.locator('#mobile-panel').evaluate('e=>e.open=true')
        page.locator('#manipulation-panel').evaluate('e=>e.open=true')
        page.locator('#mobile-wait-example-button').click()
        assert page.locator('#mobile-scenario').input_value() == '3'
        assert page.locator('#experiment-planner-select').input_value() == '1'
        assert page.locator('#experiment-controller-select').input_value() == '1'
        page.wait_for_function('''()=>{
            if (!document.querySelector('#mobile-status').textContent.includes('待機・再試行')) return false;
            document.querySelector('#mobile-pause-button').click();return true;
        }''', timeout=240000)
        page.wait_for_function("document.querySelector('#mobile-pause-button').getAttribute('aria-pressed')==='true'")
        assert page.locator('#mobile-records tr').count() == 1
        assert '計画失敗' in page.locator('#mobile-records').text_content()
        frozen = page.locator('#mobile-status').text_content()
        page.wait_for_timeout(1600)
        assert page.locator('#mobile-status').text_content() == frozen
        page.locator('#mobile-step-button').click()
        page.wait_for_function('(old)=>document.querySelector("#mobile-status").textContent!==old', arg=frozen)
        # Both trials must keep their original settings, including the queued policy.
        page.locator('#mobile-scenario').select_option('2')
        page.locator('#arm-planner').select_option('0')
        page.locator('#experiment-seed-editor').fill('43')
        page.locator('#experiment-seed-editor').dispatch_event('change')
        page.locator('#mobile-pause-button').click()
        page.wait_for_function('document.querySelectorAll("#mobile-records tr").length===2', timeout=240000)
        rows = page.locator('#mobile-records tr').all_text_contents()
        assert '再試行なし' in rows[0] and '計画失敗' in rows[0]
        assert '再試行あり' in rows[1] and '成功' in rows[1]
        assert page.locator('#mobile-records tr').nth(1).locator('td').nth(4).text_content() == '5.00 / 10'
        groups = page.locator('#mobile-summary table')
        assert groups.count() == 2
        assert '再試行なし' in groups.nth(0).text_content() and '0/1' in groups.nth(0).text_content()
        assert '再試行あり' in groups.nth(1).text_content() and groups.nth(1).text_content().count('1/1') == 4
        assert '完了 2/2試行' in page.locator('#mobile-summary').text_content()
        raw, records = download(page, args.output/'paired-results.csv')
        assert len(records) == 2 and all(len(r) == 25 and r['schema_version'] == '3' for r in records)
        assert [r['recovery'] for r in records] == ['false', 'true']
        assert all(r['expected_trials'] == '2' and r['completed_trials'] == '2' for r in records)
        assert all(r['scenario_id'] == '3' and r['seed'] == '42' and r['arm_planner'] == '2' for r in records)
        assert records[0]['result'] == '計画失敗' and records[1]['result'] == '成功'
        assert records[1]['waiting_s'] == '5.000000' and records[1]['retries'] == '10'
        assert page.locator('#mobile-replay-record option').count() == 2
        assert '再試行なし' in page.locator('#mobile-replay-record option').nth(0).text_content()
        assert '再試行あり' in page.locator('#mobile-replay-record option').nth(1).text_content()
        summary = page.locator('#mobile-summary').text_content()
        page.locator('#mobile-replay-record').select_option('0')
        seek(page, 100000)
        assert '計画失敗' in page.locator('#mobile-status').text_content()
        page.locator('#mobile-replay-record').select_option('1')
        carry_start = float(records[1]['approach_s']) + float(records[1]['grasp_s'])
        seek(page, round((carry_start+2)*30))
        assert '待機・再試行' in page.locator('#mobile-status').text_content()
        page.screenshot(path=str(args.output/'paired-waiting.png'))
        seek(page, 100000)
        assert '全4段階完了' in page.locator('#mobile-status').text_content()
        assert page.locator('#mobile-summary').text_content() == summary
        assert page.locator('#mobile-export-data').text_content() == raw.decode('utf-8-sig')
        page.set_viewport_size({'width': 390, 'height': 844})
        page.locator('#mobile-stop-button').tap()
        page.wait_for_function("document.body.getAttribute('data-experiment-active')==='false'")
        assert not errors, errors
        print('PASS: paired policies, preset, pause/step, captured conditions, separate stage statistics, CSV, both replays and mobile stop', flush=True)
        browser.close()


if __name__ == '__main__':
    main()
