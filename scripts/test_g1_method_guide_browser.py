#!/usr/bin/env python3
"""Verify method explanations, next-run selection and unchanged live/replay data."""
import argparse
from pathlib import Path
from playwright.sync_api import sync_playwright
from test_g1_bilingual_browser import english_only


FIELDS = {'experiment-planner-select': 2, 'experiment-controller-select': 2,
          'arm-planner': 3, 'arm-controller': 2}


def selected(page, field, value):
    assert page.locator(f'.method-card[data-method-field="{field}"][aria-pressed=true]').count() == 1
    card = page.locator(f'.method-card[data-method-field="{field}"][data-method-value="{value}"]')
    assert card.get_attribute('aria-pressed') == 'true'
    assert card.locator('.method-selected').is_visible()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', required=True)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as p:
        browser = p.chromium.launch(executable_path='/usr/bin/chromium', headless=True, args=[
            '--disable-dev-shm-usage', '--enable-unsafe-webgpu', '--enable-unsafe-swiftshader',
            '--use-angle=swiftshader', '--enable-features=Vulkan', '--use-vulkan=swiftshader'])
        page = browser.new_page(viewport={'width': 1280, 'height': 900}, device_scale_factor=0.5, has_touch=True)
        errors = []
        page.on('pageerror', lambda e: errors.append(str(e)))
        page.goto(args.url + '?lang=en')
        page.wait_for_function("document.querySelector('#engine-status').dataset.ready==='true'", timeout=90000)
        assert page.locator('.method-card').count() == 9
        assert not page.locator('#method-guide').evaluate('e=>e.open')
        page.locator('[data-demo-id="nav-detour"]').click()
        page.wait_for_function('''()=>{
            if(document.querySelectorAll('#experiment-records tr').length!==2)return false;
            document.querySelector('#experiment-pause-button').click();return true;
        }''', timeout=120000)
        page.wait_for_function("document.querySelector('#experiment-pause-button').getAttribute('aria-pressed')==='true'")
        page.locator('[data-method-guide-open="experiment-planner-select"]').click()
        assert page.locator('#method-guide').evaluate('e=>e.open')
        assert page.locator('#method-guide-experiment-planner-select').evaluate('e=>e===document.activeElement')
        frozen = page.locator('#experiment-status').text_content()
        nav_records = page.locator('#experiment-records').inner_html()
        counters = page.locator('input[type=hidden]').evaluate_all('es=>Object.fromEntries(es.filter(e=>/^(experiment|arm|mobile)-(run|compare|pause|step|stop|intent|replay-enabled|replay-tick)$/.test(e.id)).map(e=>[e.id,e.value]))')
        for field, count in FIELDS.items():
            for value in range(count):
                card = page.locator(f'.method-card[data-method-field="{field}"][data-method-value="{value}"]')
                card.focus()
                page.keyboard.press('Enter')
                assert page.locator('#' + field).input_value() == str(value)
                selected(page, field, value)
                assert page.locator('#experiment-status').text_content() == frozen
                assert page.locator('#experiment-records').inner_html() == nav_records
        after = page.locator('input[type=hidden]').evaluate_all('es=>Object.fromEntries(es.filter(e=>/^(experiment|arm|mobile)-(run|compare|pause|step|stop|intent|replay-enabled|replay-tick)$/.test(e.id)).map(e=>[e.id,e.value]))')
        assert after == counters
        english_only(page)
        page.screenshot(path=str(args.output / 'guide-en.png'))
        page.locator('#manipulation-panel').evaluate('e=>e.open=true')
        for field, count in FIELDS.items():
            page.locator('#' + field).select_option('0')
            selected(page, field, 0)
        page.locator('#language-select').select_option('ja')
        page.wait_for_function("document.querySelector('#method-guide').textContent.includes('次の実行に選択中')")
        for field in FIELDS:
            selected(page, field, 0)
        page.set_viewport_size({'width': 390, 'height': 844})
        page.locator('.method-card[data-method-field="arm-planner"][data-method-value="2"]').tap()
        selected(page, 'arm-planner', 2)
        assert page.locator('#arm-planner').input_value() == '2'
        assert page.evaluate('document.documentElement.scrollWidth<=window.innerWidth')
        page.screenshot(path=str(args.output / 'guide-ja-mobile.png'))
        # Shared restoration must synchronize cards, without starting a run.
        page.set_viewport_size({'width': 1280, 'height': 900})
        page.locator('#share-panel').evaluate('e=>e.open=true')
        page.locator('#share-mode').select_option('mobile-run')
        page.locator('#share-create').click()
        shared = page.locator('#share-url').input_value()
        page.locator('#arm-planner').select_option('0')
        page.evaluate('(url)=>{location.hash=new URL(url).hash;}', shared)
        page.wait_for_function("document.querySelector('#arm-planner').value==='2'")
        selected(page, 'arm-planner', 2)
        assert page.locator('#experiment-pause-button').get_attribute('aria-pressed') == 'true'
        page.locator('#language-select').select_option('en')
        page.wait_for_function("document.querySelector('#experiment-status').textContent.includes('Paused')")
        assert page.locator('#experiment-status').text_content() == frozen
        assert page.locator('#experiment-records').inner_html() == nav_records
        print('PASS: all 9 methods, keyboard/touch selection, native form synchronization, shared restore, ja/en, focus and unchanged paused experiment/counters/results', flush=True)
        # Retain completed evidence while selecting methods during a real replay.
        page.locator('#experiment-manual-button').click()
        page.locator('#language-select').select_option('ja')
        page.locator('#mobile-panel').evaluate('e=>e.open=true')
        page.locator('#experiment-planner-select').select_option('0')
        page.locator('#mobile-run-button').click()
        page.wait_for_function("document.querySelector('#mobile-replay-record option')!==null", timeout=120000)
        page.locator('#mobile-replay-record').dispatch_event('change')
        page.wait_for_function("document.querySelector('#mobile-status').textContent.includes('リプレイ')")
        replay = page.locator('#mobile-status').text_content()
        csv = page.locator('#mobile-export-data').text_content()
        page.locator('#manipulation-panel').evaluate('e=>e.open=true')
        page.locator('[data-method-guide-open="arm-planner"]').click()
        page.locator('.method-card[data-method-field="arm-planner"][data-method-value="1"]').click()
        assert page.locator('#mobile-replay-enabled').input_value() == '1'
        assert page.locator('#mobile-status').text_content() == replay
        assert page.locator('#mobile-export-data').text_content() == csv
        assert not errors, errors
        print('PASS: method selection leaves saved delivery replay and CSV unchanged', flush=True)
        browser.close()


if __name__ == '__main__':
    main()
