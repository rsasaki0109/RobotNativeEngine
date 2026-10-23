#!/usr/bin/env python3
"""Check timeline intervals, failure jumps and replay synchronization in real WASM."""
import argparse
from pathlib import Path
from playwright.sync_api import sync_playwright
from test_g1_mobile_wait_browser import download


def choose(page, button, keyboard=False):
    tick = int(button.get_attribute('data-mobile-tick'))
    record = button.get_attribute('data-mobile-record')
    if keyboard:
        button.focus()
        page.keyboard.press('Enter')
    else:
        button.click()
    page.wait_for_function('''tick=>document.querySelector('#mobile-status').textContent.includes('リプレイ') &&
        document.querySelector('#mobile-status').textContent.includes((tick/30).toFixed(2)+' s') &&
        Number(document.querySelector('#mobile-replay-tick').value)===tick''', arg=tick)
    assert page.locator('#mobile-replay-record').input_value() == record
    assert page.locator('#mobile-replay-time').evaluate('e=>e.value') == f'{tick/30:.2f} s'


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
        page.locator('#mobile-wait-example-button').click()
        page.wait_for_function('''()=>{
            if (!document.querySelector('#mobile-status').textContent.includes('待機・再試行')) return false;
            document.querySelector('#mobile-pause-button').click();return true;
        }''', timeout=240000)
        page.wait_for_function("document.querySelector('#mobile-pause-button').getAttribute('aria-pressed')==='true'")
        assert page.locator('#mobile-timeline .timeline-row').count() == 1
        assert page.locator('#mobile-timeline button:not(:disabled)').count() == 0
        assert page.locator('#mobile-timeline button:disabled').count() > 0
        page.locator('#mobile-pause-button').click()
        page.wait_for_function('document.querySelectorAll("#mobile-timeline .timeline-row").length===2 && document.querySelectorAll("#mobile-timeline button:disabled").length===0', timeout=240000)
        raw, records = download(page, args.output/'timeline-results.csv')
        assert records[0]['result'] == '計画失敗' and records[1]['result'] == '成功'
        timeline = page.locator('#mobile-timeline').inner_html()
        summary = page.locator('#mobile-summary').text_content()
        choose(page, page.locator('.timeline-terminal[data-mobile-record="0"]'))
        assert '計画失敗' in page.locator('#mobile-status').text_content()
        assert page.locator('#mobile-replay-frame').input_value() == page.locator('#mobile-replay-frame').get_attribute('max')
        choose(page, page.locator('.timeline-segment[data-mobile-record="1"][data-phase="waiting"]'))
        assert '待機・再試行' in page.locator('#mobile-status').text_content()
        assert '物体保持中' in page.locator('#mobile-status').text_content()
        choose(page, page.locator('.timeline-segment[data-mobile-record="1"][data-phase="2"]'))
        assert '待機・再試行' not in page.locator('#mobile-status').text_content()
        choose(page, page.locator('.timeline-segment[data-mobile-record="1"][data-phase="3"]'), keyboard=True)
        assert '配置 ·' in page.locator('#mobile-status').text_content()
        choose(page, page.locator('.timeline-terminal[data-mobile-record="1"]'))
        assert '全4段階完了' in page.locator('#mobile-status').text_content()
        page.screenshot(path=str(args.output/'timeline.png'))
        assert page.locator('#mobile-timeline').inner_html() == timeline
        assert page.locator('#mobile-summary').text_content() == summary
        assert page.locator('#mobile-export-data').text_content() == raw.decode('utf-8-sig')
        page.set_viewport_size({'width': 390, 'height': 844})
        failure = page.locator('.timeline-terminal[data-mobile-record="0"]')
        tick = failure.get_attribute('data-mobile-tick')
        failure.tap()
        page.wait_for_function('tick=>document.querySelector("#mobile-replay-tick").value===tick && document.querySelector("#mobile-status").textContent.includes("計画失敗")', arg=tick)
        page.screenshot(path=str(args.output/'timeline-mobile.png'))
        page.locator('#mobile-stop-button').tap()
        page.wait_for_function("document.body.getAttribute('data-experiment-active')==='false'")
        assert not errors, errors
        print('PASS: partial timeline disabled, failure/wait/carry/place/end jumps, slider synchronization, keyboard/mobile and immutable records', flush=True)
        browser.close()


if __name__ == '__main__':
    main()
