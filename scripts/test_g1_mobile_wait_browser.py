#!/usr/bin/env python3
"""Verify temporary delivery closure with recovery disabled/enabled in real WASM."""
import argparse
import csv
import io
from pathlib import Path
from playwright.sync_api import sync_playwright
from test_g1_mobile_browser import seek


def download(page, destination):
    with page.expect_download() as info:
        page.locator('#mobile-export-button').click()
    info.value.save_as(destination)
    raw = destination.read_bytes()
    assert raw.startswith(b'\xef\xbb\xbf')
    return raw, list(csv.DictReader(io.StringIO(raw.decode('utf-8-sig'))))


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
        page.locator('#mobile-scenario').select_option('3')
        page.locator('#mobile-compare-button').click()
        page.wait_for_function('document.querySelectorAll("#mobile-records tr").length===4', timeout=240000)
        _, off = download(page, args.output/'recovery-off.csv')
        assert len(off) == 4 and all(r['scenario_id'] == '3' and r['recovery'] == 'false' for r in off)
        assert all(r['result'] == '計画失敗' and r['completed_stages'] == '2' for r in off[2:])
        assert all(r['waiting_s'] == '0.000000' and r['retries'] == '0' for r in off)
        print('PASS: temporary closure fails during carry without recovery', flush=True)
        page.locator('#mobile-recovery-toggle').check()
        page.locator('#mobile-compare-button').click()
        page.wait_for_function('''()=>{
            if (!document.querySelector('#mobile-status').textContent.includes('待機・再試行')) return false;
            document.querySelector('#mobile-pause-button').click();return true;
        }''', timeout=240000)
        page.wait_for_function("document.querySelector('#mobile-pause-button').getAttribute('aria-pressed')==='true'")
        frozen = page.locator('#mobile-status').text_content()
        position = page.locator('#engine-position').text_content()
        page.wait_for_timeout(1800)
        assert page.locator('#mobile-status').text_content() == frozen
        assert page.locator('#engine-position').text_content() == position
        page.locator('#mobile-step-button').click()
        page.wait_for_function('(old)=>document.querySelector("#mobile-status").textContent!==old', arg=frozen)
        assert page.locator('#engine-position').text_content() == position
        # Changing this control while paused must not change the captured policy.
        page.locator('#mobile-recovery-toggle').uncheck()
        page.locator('#mobile-pause-button').click()
        page.wait_for_function('document.querySelectorAll("#mobile-records tr").length===4 && document.querySelector("#mobile-export-data").textContent.includes(",true,5.000000,10")', timeout=240000)
        raw, on = download(page, args.output/'recovery-on.csv')
        assert len(on) == 4 and all(len(r) == 25 and r['schema_version'] == '3' for r in on)
        assert all(r['recovery'] == 'true' and r['scenario_id'] == '3' for r in on)
        assert all(r['completed_stages'] == '0' and r['waiting_s'] == '0.000000' for r in on[:2])
        assert all(r['result'] == '成功' and r['waiting_s'] == '5.000000' and r['retries'] == '10' for r in on[2:])
        page.locator('#mobile-replay-record').select_option('2')
        carry_start = float(on[2]['approach_s']) + float(on[2]['grasp_s'])
        seek(page, round((carry_start+2)*30))
        assert '待機・再試行' in page.locator('#mobile-status').text_content()
        assert '物体保持中' in page.locator('#mobile-status').text_content()
        page.screenshot(path=str(args.output/'waiting.png'))
        seek(page, round((carry_start+6)*30))
        status = page.locator('#mobile-status').text_content()
        assert '待機・再試行' not in status and '再試行 10回' in status, status
        page.screenshot(path=str(args.output/'reopened.png'))
        seek(page, 100000)
        assert '全4段階完了' in page.locator('#mobile-status').text_content()
        assert page.locator('#mobile-export-data').text_content() == raw.decode('utf-8-sig')
        page.set_viewport_size({'width': 390, 'height': 844})
        page.locator('#mobile-stop-button').tap()
        page.wait_for_function("document.body.getAttribute('data-experiment-active')==='false'")
        assert not errors, errors
        print('PASS: paused waiting/step, captured recovery policy, 5-second/10-retry success, CSV, replay and mobile stop', flush=True)
        browser.close()


if __name__ == '__main__':
    main()
