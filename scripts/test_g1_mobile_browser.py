#!/usr/bin/env python3
"""Verify the real WASM delivery task, synchronized replay and CSV download."""
import argparse
import csv
import io
from pathlib import Path
from playwright.sync_api import sync_playwright


def seek(page, tick):
    page.locator('#mobile-replay-frame').evaluate('''(el,tick)=>{
        const times=document.querySelector('#mobile-replay-record').selectedOptions[0].dataset.times.split(',').map(Number);
        let index=0;
        for(let i=0;i<times.length;i++) if(Math.abs(times[i]-tick)<Math.abs(times[index]-tick)) index=i;
        el.value=String(index);el.dispatchEvent(new Event('input',{bubbles:true}));
    }''', tick)
    page.wait_for_function('''()=>document.querySelector('#mobile-status').textContent.includes('リプレイ') &&
        document.querySelector('#mobile-status').textContent.includes(document.querySelector('#mobile-replay-time').value)''')


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
        page.locator('#experiment-planner-select').select_option('1')
        page.locator('#mobile-example-button').click()
        page.wait_for_function("document.querySelector('#mobile-status').textContent.includes('実行中')")
        page.locator('#mobile-pause-button').click()
        page.wait_for_function("document.querySelector('#mobile-pause-button').getAttribute('aria-pressed')==='true'")
        assert page.locator('#experiment-controller-select').input_value() == '1'
        assert page.locator('#arm-planner').input_value() == '2'
        frozen = page.locator('#mobile-status').text_content()
        page.wait_for_timeout(1800)
        assert page.locator('#mobile-status').text_content() == frozen
        page.locator('#mobile-step-button').click()
        page.wait_for_function('(old)=>document.querySelector("#mobile-status").textContent!==old', arg=frozen)
        page.locator('#mobile-compare-button').click()
        page.wait_for_function('document.querySelectorAll("#mobile-records tr").length>=2')
        # Captured methods must remain fixed despite subsequent selection changes.
        page.locator('#arm-planner').select_option('0')
        page.wait_for_function('document.querySelectorAll("#mobile-records tr").length===4', timeout=240000)
        rows = page.locator('#mobile-records tr').all_text_contents()
        assert all('計画失敗' in row and '0/4' in row for row in rows[:2]), rows
        assert all('成功' in row and '4/4' in row and 'RRT-Connect' in row for row in rows[2:]), rows
        summary = page.locator('#mobile-summary').text_content()
        assert '評価完了' in summary and '2/4' in summary and summary.count('2/2') == 3, summary
        assert page.locator('#mobile-replay-record option').count() == 4
        with page.expect_download() as info:
            page.locator('#mobile-export-button').click()
        download = info.value
        assert download.suggested_filename == 'rne-mobile-results.csv'
        destination = args.output/'results.csv'
        download.save_as(destination)
        raw = destination.read_bytes()
        assert raw.startswith(b'\xef\xbb\xbf')
        records = list(csv.DictReader(io.StringIO(raw.decode('utf-8-sig'))))
        assert len(records) == 4 and all(len(row) == 22 for row in records)
        assert all(row['schema_version'] == '2' and row['scenario_id'] == '0' for row in records)
        assert all(row['arm_planner'] == '2' and row['batch_state'] == 'completed' for row in records)
        assert all(row['carry_s'] == '' and row['place_s'] == '' for row in records[:2])
        assert all(float(row['final_error_m']) < .025 for row in records[2:])
        page.locator('#mobile-replay-record').select_option('2')
        held_tick = round((float(records[2]['approach_s']) + float(records[2]['grasp_s']) + 1.0)*30)
        seek(page, held_tick)
        assert '物体保持中' in page.locator('#mobile-status').text_content()
        held_position = page.locator('#engine-position').text_content()
        page.screenshot(path=str(args.output/'delivery-carry.png'))
        seek(page, 0)
        assert '未保持' in page.locator('#mobile-status').text_content()
        assert page.locator('#engine-position').text_content() != held_position
        seek(page, 100000)
        assert '全4段階完了' in page.locator('#mobile-status').text_content()
        assert '未保持' in page.locator('#mobile-status').text_content()
        assert page.locator('#mobile-summary').text_content() == summary
        assert page.locator('#mobile-export-data').text_content() == raw.decode('utf-8-sig')
        page.set_viewport_size({'width': 390, 'height': 844})
        page.locator('#mobile-step-button').tap()
        page.wait_for_function("!document.querySelector('#mobile-status').textContent.includes('リプレイ')")
        page.screenshot(path=str(args.output/'delivery-mobile.png'))
        page.locator('#experiment-manual-button').tap()
        page.wait_for_function("document.body.getAttribute('data-experiment-active')==='false'")
        assert not errors, errors
        print('PASS: delivery four methods, stage outcomes, pause/step, captured arm config, CSV, held/placed replay and mobile manual takeover')
        browser.close()


if __name__ == '__main__':
    main()
