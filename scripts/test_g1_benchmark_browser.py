#!/usr/bin/env python3
"""Check the real WASM multi-seed benchmark UI, interruption and replay."""
import argparse
from pathlib import Path
from playwright.sync_api import sync_playwright


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
        page = browser.new_page(viewport={'width': 1280, 'height': 900})
        errors = []
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.on('console', lambda message: errors.append(message.text) if 'GPUValidationError' in message.text else None)
        page.goto(args.url)
        page.wait_for_function("document.querySelector('#engine-status').textContent.startsWith('操作できます')", timeout=90000)
        page.locator('#manipulation-panel').evaluate('e=>e.open=true')
        page.locator('#arm-task').select_option('0')
        page.locator('#arm-benchmark-button').click()
        page.wait_for_function("document.querySelector('#arm-benchmark').textContent.includes('/18')")
        page.locator('#arm-pause-button').click()
        page.wait_for_function("document.querySelector('#arm-benchmark').textContent.includes('一時停止')")
        frozen = page.locator('#arm-status').text_content()
        page.wait_for_timeout(1600)
        assert page.locator('#arm-status').text_content() == frozen
        page.locator('#arm-step-button').click()
        page.wait_for_function('(old)=>document.querySelector("#arm-status").textContent!==old', arg=frozen)
        page.locator('#arm-stop-button').click()
        page.wait_for_function("document.querySelector('#arm-benchmark').textContent.includes('中断')")
        page.locator('#arm-task').select_option('3')
        page.locator('#arm-benchmark-button').click()
        page.wait_for_function('document.querySelectorAll("#arm-records tr").length===18', timeout=60000)
        summary = page.locator('#arm-benchmark').text_content()
        assert '評価完了' in summary and summary.count('0/3') == 6, summary
        assert summary.count('—') == 12 and 'NaN' not in summary, summary
        assert page.locator('#arm-replay-record option').count() == 18
        assert 'seed' in page.locator('#arm-replay-record option').nth(6).text_content()
        page.locator('#arm-replay-record').select_option('0')
        page.wait_for_function("document.querySelector('#arm-status').textContent.includes('リプレイ')")
        assert page.locator('#arm-benchmark').text_content() == summary
        page.screenshot(path=str(args.output/'benchmark.png'))
        page.locator('#arm-compare-button').click()
        page.wait_for_function('document.querySelectorAll("#arm-records tr").length===6')
        assert page.locator('#arm-benchmark').text_content() == ''
        assert not errors, errors
        print('PASS: benchmark pause, step, interruption, 18 failures, summary, replay and normal comparison')
        browser.close()


if __name__ == '__main__':
    main()
