#!/usr/bin/env python3
"""Exercise all 24 delivery pairs, captured settings and matrix replay in WASM."""
import argparse
from pathlib import Path
from playwright.sync_api import sync_playwright
from test_g1_mobile_wait_browser import download
from test_g1_mobile_diagnostics_browser import choose
from test_g1_bilingual_browser import english_only


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
        page = browser.new_page(viewport={'width': 1280, 'height': 900}, device_scale_factor=0.5, has_touch=True)
        errors = []
        page.on('pageerror', lambda e: errors.append(str(e)))
        page.on('console', lambda m: errors.append(m.text) if 'GPUValidationError' in m.text else None)
        page.goto(args.url + '?lang=en')
        page.wait_for_function("document.querySelector('#engine-status').dataset.ready==='true'", timeout=90000)
        page.locator('#experiment-panel').evaluate('e=>e.open=true')
        page.locator('#manipulation-panel').evaluate('e=>e.open=true')
        page.locator('#mobile-panel').evaluate('e=>e.open=true')
        page.locator('#experiment-seed-editor').fill('42')
        page.locator('#experiment-seed-editor').dispatch_event('change')
        page.locator('#mobile-matrix-button').focus()
        page.keyboard.press('Enter')
        assert page.locator('#mobile-intent').input_value() == '6'
        page.wait_for_function('''()=>{
            if(document.querySelectorAll('#mobile-records tr').length!==12)return false;
            document.querySelector('#mobile-pause-button').click();return true;
        }''', timeout=120000)
        page.wait_for_function("document.querySelector('#mobile-pause-button').getAttribute('aria-pressed')==='true'")
        matrix = page.locator('.mobile-matrix-table')
        assert matrix.locator('tbody tr').count() == 4
        assert matrix.locator('[data-matrix-state="failure"]').count() == 12
        assert matrix.locator('[data-matrix-state="pending"]').count() == 11
        assert matrix.locator('[data-matrix-state="running"]').count() == 1
        assert matrix.locator('button:enabled').count() == 0
        frozen = page.locator('#mobile-status').text_content()
        page.wait_for_timeout(1500)
        assert page.locator('#mobile-status').text_content() == frozen
        # Form edits must not leak into the queued 24 trials.
        page.locator('#mobile-scenario').select_option('2')
        page.locator('#mobile-recovery-toggle').check()
        page.locator('#experiment-seed-editor').fill('99')
        page.locator('#experiment-seed-editor').dispatch_event('change')
        page.locator('#mobile-pause-button').click()
        page.wait_for_function('document.querySelectorAll(".mobile-matrix-table button:enabled").length===24', timeout=360000)
        raw, records = download(page, args.output / 'matrix.csv')
        assert len(records) == 24
        assert [(int(r['nav_planner']), int(r['nav_controller']), int(r['arm_planner']), int(r['arm_controller'])) for r in records] == [
            (np, nc, ap, ac) for np in range(2) for nc in range(2) for ap in range(3) for ac in range(2)]
        assert all(r['seed'] == '42' and r['scenario_id'] == '0' and r['recovery'] == 'false' for r in records)
        assert all(r['expected_trials'] == '24' and r['completed_trials'] == '24' and r['batch_state'] == 'completed' for r in records)
        assert sum(r['result'] == '成功' for r in records) == 6
        assert matrix.locator('[data-matrix-state="success"]').count() == 6
        assert matrix.locator('[data-matrix-state="failure"]').count() == 18
        assert matrix.locator('[data-matrix-state="pending"]').count() == 0
        summary = page.locator('#mobile-summary').inner_html()
        for index in [0, 15, 23]:
            choose(page, matrix.locator(f'button[data-mobile-record="{index}"]'), keyboard=index == 15)
        assert 'All 4 stages completed' in page.locator('#mobile-status').text_content()
        page.locator('#mobile-filter-outcome').select_option('failure')
        assert matrix.locator('button:enabled').count() == 24
        assert page.locator('#mobile-export-data').text_content() == raw.decode('utf-8-sig')
        assert page.locator('#mobile-summary').inner_html() == summary
        english_only(page)
        page.screenshot(path=str(args.output / 'matrix-en.png'))
        page.locator('#language-select').select_option('ja')
        page.wait_for_function("document.querySelector('.mobile-matrix-table').textContent.includes('未実行')===false && document.querySelector('.mobile-matrix-table caption').textContent.includes('運搬24試行')")
        page.set_viewport_size({'width': 390, 'height': 844})
        choose(page, matrix.locator('button[data-mobile-record="23"]'))
        scroll = page.locator('.mobile-matrix-scroll')
        assert scroll.evaluate('e=>e.scrollWidth>e.clientWidth')
        assert page.evaluate('document.documentElement.scrollWidth<=window.innerWidth')
        page.screenshot(path=str(args.output / 'matrix-ja-mobile.png'))
        print('PASS: 24 real trials, captured conditions, mixed results, pending states, pause, original-index replay, keyboard/mobile and ja/en', flush=True)
        # Restore conditions through the new share mode without auto-starting.
        page.locator('#share-panel').evaluate('e=>e.open=true')
        page.locator('#share-mode').select_option('mobile-matrix')
        page.locator('#share-create').click()
        shared = page.locator('#share-url').input_value()
        assert page.evaluate('(url)=>RNEExperimentShare.decode(new URL(url).hash).mode', shared) == 'mobile-matrix'
        page.evaluate('(url)=>{location.hash=new URL(url).hash;}', shared)
        page.wait_for_function("!document.querySelector('#share-start').disabled")
        assert page.locator('#mobile-export-data').text_content() == raw.decode('utf-8-sig')
        page.locator('#share-start').click()
        page.wait_for_function('''()=>{
            if(!document.querySelectorAll('.mobile-matrix-table [data-matrix-state="running"]').length)return false;
            document.querySelector('#mobile-pause-button').click();return true;
        }''', timeout=120000)
        page.wait_for_function("document.querySelector('#mobile-pause-button').getAttribute('aria-pressed')==='true'")
        assert page.locator('#mobile-intent').input_value() == '6'
        page.locator('#mobile-stop-button').click()
        page.wait_for_function("document.querySelector('#mobile-summary').textContent.includes('中断')")
        assert matrix.locator('[data-matrix-state]').count() == 24
        assert matrix.locator('button:enabled').count() == 0
        assert matrix.locator('[data-matrix-state="pending"]').count() > 0
        _, partial = download(page, args.output / 'interrupted.csv')
        assert len(partial) < 24
        assert all(r['seed'] == '99' and r['scenario_id'] == '2' and r['recovery'] == 'true' and r['batch_state'] == 'interrupted' for r in partial)
        assert not errors, errors
        print('PASS: shared matrix launch, preview-only restore and read-only interrupted matrix', flush=True)
        browser.close()


if __name__ == '__main__':
    main()
