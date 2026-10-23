#!/usr/bin/env python3
"""Launch gallery presets using real WASM controls, keyboard and mobile layout."""
import argparse
from pathlib import Path
from playwright.sync_api import sync_playwright
from test_g1_mobile_wait_browser import download


def back(page, domain):
    selector = {'nav': '#experiment-panel > [data-demo-back]', 'arm': '#manipulation-panel [data-demo-back]', 'mobile': '#mobile-panel [data-demo-back]'}[domain]
    page.locator(selector).click()
    assert page.locator('#demo-gallery').evaluate('e=>e.open')
    assert page.evaluate('document.activeElement.matches("[data-demo-id]")')


def launch(page, name):
    page.locator(f'[data-demo-id="{name}"]').click()
    assert not page.locator('#demo-gallery').evaluate('e=>e.open')


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
        assert page.locator('[data-demo-id]').count() == 7
        if not page.locator('#engine-status').text_content().startswith('操作できます'):
            assert page.locator('[data-demo-id]:disabled').count() == 7
        page.wait_for_function("document.querySelector('#engine-status').textContent.startsWith('操作できます')", timeout=90000)
        page.wait_for_function('document.querySelectorAll("[data-demo-id]:disabled").length===0')
        page.locator('#experiment-panel').evaluate('e=>e.open=true')
        page.locator('#experiment-seed-editor').fill('99')
        page.locator('#experiment-seed-editor').dispatch_event('change')
        page.locator('[data-trace="2"]').uncheck()
        launch(page, 'nav-detour')
        page.wait_for_function("document.querySelector('#experiment-status').textContent.includes('障害物を迂回')")
        assert page.locator('#experiment-seed').input_value() == '42'
        assert page.locator('#experiment-task').input_value() == '1'
        assert page.locator('[data-trace]:checked').count() == 4
        assert not page.locator('#manipulation-panel').evaluate('e=>e.open')
        assert not page.locator('#mobile-panel').evaluate('e=>e.open')
        page.wait_for_function('document.querySelectorAll("#experiment-records tr").length===4', timeout=240000)
        rows = page.locator('#experiment-records tr').all_text_contents()
        assert any('成功' in row for row in rows) and any('計画失敗' in row for row in rows), rows
        back(page, 'nav')
        launch(page, 'nav-crossing')
        page.wait_for_function("document.querySelector('#experiment-status').textContent.includes('横切る障害物')")
        assert page.locator('#experiment-recovery').input_value() == '1'
        back(page, 'nav')
        page.locator('[data-demo-id="arm-obstacle"]').focus()
        page.keyboard.press('Enter')
        page.wait_for_function('document.querySelectorAll("#arm-records tr").length===6', timeout=240000)
        rows = page.locator('#arm-records tr').all_text_contents()
        assert sum('成功' in row for row in rows) == 2 and sum('接触' in row for row in rows) == 4, rows
        assert page.locator('#manipulation-panel').evaluate('e=>e.open')
        assert page.locator('#arm-task').input_value() == '2'
        back(page, 'arm')
        page.locator('#manipulation-panel').evaluate('e=>e.open=false')
        page.locator('#demo-result-link').click()
        assert page.locator('#manipulation-panel').evaluate('e=>e.open')
        assert page.evaluate('document.activeElement.id') == 'arm-status'
        assert page.locator('#arm-records tr').count() == 6
        back(page, 'arm')
        launch(page, 'delivery')
        page.wait_for_function("document.querySelector('#mobile-status').textContent.startsWith('標準の運搬')")
        assert page.locator('#mobile-scenario').input_value() == '0'
        assert page.locator('#mobile-recovery').input_value() == '0'
        back(page, 'mobile')
        launch(page, 'closure')
        page.wait_for_function("document.querySelector('#mobile-status').textContent.startsWith('把持後に運搬経路が閉鎖')")
        assert page.locator('#mobile-scenario').input_value() == '1'
        back(page, 'mobile')
        launch(page, 'recovery-pair')
        page.wait_for_function('''()=>{
            const rows=document.querySelector('#mobile-export-data').textContent.trim().split(String.fromCharCode(10)).slice(1);
            return rows.length===2 && rows.every(row=>row.split(',')[3]==='2' && row.split(',')[20]==='3');
        }''', timeout=240000)
        _, records = download(page, args.output/'gallery-recovery.csv')
        assert [r['recovery'] for r in records] == ['false', 'true']
        assert records[0]['result'] == '計画失敗' and records[1]['result'] == '成功'
        assert all(r['seed'] == '42' and r['arm_planner'] == '2' for r in records)
        assert '待って運搬を再開' in page.locator('#demo-launch-status').text_content()
        back(page, 'mobile')
        page.set_viewport_size({'width': 390, 'height': 844})
        page.locator('[data-demo-id="recovery-pair"]').scroll_into_view_if_needed()
        box = page.locator('[data-demo-id="recovery-pair"]').bounding_box()
        assert box and box['x'] >= 0 and box['x']+box['width'] <= 390, box
        page.screenshot(path=str(args.output/'gallery-mobile.png'))
        page.locator('[data-demo-id="nav-crossing"]').tap()
        page.wait_for_function("document.querySelector('#experiment-status').textContent.includes('横切る障害物')")
        assert not page.locator('#mobile-panel').evaluate('e=>e.open')
        page.locator('#experiment-manual-button').tap()
        page.wait_for_function("document.body.getAttribute('data-experiment-active')==='false'")
        assert not errors, errors
        print('PASS: six presets, dirty settings reset, native navigation/arm/recovery results, keyboard, progress/back links and mobile switching', flush=True)
        browser.close()


if __name__ == '__main__':
    main()
