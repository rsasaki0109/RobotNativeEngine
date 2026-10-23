#!/usr/bin/env python3
"""Check closed-route and high-shelf delivery scenarios in the actual WASM viewer."""
import argparse
import csv
import io
from pathlib import Path
from playwright.sync_api import sync_playwright
from test_g1_mobile_browser import seek


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
        page.locator('#arm-planner').select_option('2')
        page.locator('#arm-controller').select_option('0')
        for scenario, stage, reason in [('1', '2', '計画失敗'), ('2', '3', 'IK失敗')]:
            page.locator('#mobile-scenario').select_option(scenario)
            page.locator('#mobile-compare-button').click()
            page.wait_for_function("""scenario => {
                const rows=document.querySelector('#mobile-export-data').textContent.trim().split(String.fromCharCode(10)).slice(1);
                return rows.length>=2 && rows.every(row=>row.split(',')[20]===scenario);
            }""", arg=scenario)
            # A changed selector applies to the next run only.
            page.locator('#mobile-scenario').select_option('0')
            page.wait_for_function('document.querySelectorAll("#mobile-records tr").length===4', timeout=240000)
            with page.expect_download() as info:
                page.locator('#mobile-export-button').click()
            path = args.output/f'scenario-{scenario}.csv'
            info.value.save_as(path)
            raw = path.read_bytes()
            assert raw.startswith(b'\xef\xbb\xbf')
            records = list(csv.DictReader(io.StringIO(raw.decode('utf-8-sig'))))
            assert len(records) == 4 and all(len(r) == 22 for r in records)
            assert all(r['schema_version'] == '2' and r['scenario_id'] == scenario for r in records)
            assert all(r['batch_state'] == 'completed' and r['completed_trials'] == '4' for r in records)
            assert all(r['completed_stages'] == '0' and r['grasp_s'] == '' for r in records[:2])
            assert all(r['completed_stages'] == stage and r['result'] == reason for r in records[2:])
            if scenario == '1':
                assert all(r['place_s'] == '' for r in records[2:])
            else:
                assert all(float(r['carry_s']) > 0 and float(r['final_error_m']) > 1 for r in records[2:])
            snapshot = page.locator('#mobile-export-data').text_content()
            page.locator('#mobile-replay-record').select_option('2')
            seek(page, 0)
            assert '未保持' in page.locator('#mobile-status').text_content()
            if scenario == '1':
                page.screenshot(path=str(args.output/'gate-open.png'))
            seek(page, 100000)
            status = page.locator('#mobile-status').text_content()
            assert reason in status and '物体保持中' in status and 'リプレイ' in status, status
            assert records[2]['scenario'] in status
            assert page.locator('#mobile-export-data').text_content() == snapshot
            page.screenshot(path=str(args.output/f'scenario-{scenario}-failure.png'))
            print(f'PASS: scenario {scenario}, captured condition, stage {stage}, CSV and replay', flush=True)
        # The success preset must explicitly restore the standard condition.
        page.locator('#mobile-scenario').select_option('2')
        page.locator('#mobile-example-button').click()
        page.wait_for_function("document.querySelector('#mobile-status').textContent.startsWith('標準の運搬') && document.querySelector('#mobile-status').textContent.includes('実行中')")
        assert page.locator('#mobile-scenario').input_value() == '0'
        page.locator('#mobile-pause-button').click()
        page.wait_for_function("document.querySelector('#mobile-pause-button').getAttribute('aria-pressed')==='true'")
        page.set_viewport_size({'width': 390, 'height': 844})
        page.locator('#mobile-stop-button').tap()
        page.wait_for_function("document.body.getAttribute('data-experiment-active')==='false'")
        assert not errors, errors
        print('PASS: standard preset resets scenario, pause and mobile stop', flush=True)
        browser.close()


if __name__ == '__main__':
    main()
