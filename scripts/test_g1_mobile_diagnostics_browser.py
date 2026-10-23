#!/usr/bin/env python3
"""Check delivery diagnostics, display filters and saved-frame replay in real WASM."""
import argparse
from pathlib import Path
from playwright.sync_api import sync_playwright
from test_g1_mobile_wait_browser import download
from test_g1_bilingual_browser import english_only


def choose(page, button, keyboard=False):
    tick=int(button.get_attribute('data-mobile-tick'))
    record=button.get_attribute('data-mobile-record')
    if keyboard:
        button.focus()
        page.keyboard.press('Enter')
    else:
        button.click()
    page.wait_for_function('''tick=>{
        const text=document.querySelector('#mobile-status').textContent;
        return (text.includes('リプレイ')||text.includes('Replay')) && text.includes((tick/30).toFixed(2)+' s') && Number(document.querySelector('#mobile-replay-tick').value)===tick;
    }''',arg=tick)
    assert page.locator('#mobile-replay-record').input_value()==record


def ids(page, selector):
    return page.locator(selector+' [data-mobile-index]:not([hidden])').evaluate_all('es=>es.map(e=>Number(e.dataset.mobileIndex))')


def assert_ids(page, expected):
    for selector in ['#mobile-records','#mobile-timeline','#mobile-diagnostics']:
        assert ids(page,selector)==expected,(selector,ids(page,selector),expected)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    with sync_playwright() as p:
        browser=p.chromium.launch(executable_path='/usr/bin/chromium',headless=True,args=[
            '--disable-dev-shm-usage','--enable-unsafe-webgpu','--enable-unsafe-swiftshader',
            '--use-angle=swiftshader','--enable-features=Vulkan','--use-vulkan=swiftshader'])
        page=browser.new_page(viewport={'width':1280,'height':900},device_scale_factor=0.5,has_touch=True)
        errors=[]
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.goto(args.url+'?lang=en')
        page.wait_for_function("document.querySelector('#engine-status').dataset.ready==='true'",timeout=90000)
        assert 'No completed trials' in page.locator('#mobile-filter-count').text_content()
        page.locator('[data-demo-id="delivery-arm"]').click()
        page.wait_for_function('''()=>{
            if(document.querySelectorAll('#mobile-records tr').length!==1)return false;
            document.querySelector('#mobile-pause-button').click();return true;
        }''',timeout=120000)
        page.wait_for_function("document.querySelector('#mobile-pause-button').getAttribute('aria-pressed')==='true'")
        page.locator('#mobile-diagnostic-panel').evaluate('e=>e.open=true')
        assert page.locator('#mobile-diagnostics article').count()==1
        assert page.locator('#mobile-diagnostics button:enabled').count()==0
        page.locator('#mobile-filter-outcome').select_option('success')
        assert_ids(page,[])
        assert page.locator('#mobile-filter-empty').is_visible()
        assert page.locator('#mobile-pause-button').get_attribute('aria-pressed')=='true'
        page.locator('#mobile-filter-reset').click()
        assert_ids(page,[0])
        page.locator('#mobile-pause-button').click()
        page.wait_for_function('document.querySelectorAll("#mobile-diagnostics article").length===6 && document.querySelectorAll("#mobile-diagnostics button:disabled").length===0',timeout=300000)
        raw,records=download(page,args.output/'six-methods.csv')
        assert len(records)==6
        summary=page.locator('#mobile-summary').inner_html()
        replay_options=page.locator('#mobile-replay-record option').count()
        page.locator('#mobile-filter-outcome').select_option('failure')
        page.locator('#mobile-filter-stage').select_option('1')
        page.locator('#mobile-filter-seed').select_option('42')
        assert_ids(page,[0,1,3])
        assert page.locator('#mobile-replay-enabled').input_value()=='0'
        assert page.locator('#mobile-replay-record option').count()==replay_options==6
        assert '3/6' in page.locator('#mobile-filter-count').text_content()
        # Filtering must retain original record 3, rather than renumbering visible trial 2.
        card=page.locator('#mobile-diagnostics [data-mobile-index="3"]')
        choose(page,card.locator('button').nth(0))
        before=int(card.locator('button').nth(0).get_attribute('data-mobile-tick'))
        end=int(card.locator('button').nth(1).get_attribute('data-mobile-tick'))
        assert 30<=end-before<=32
        choose(page,card.locator('button').nth(1),keyboard=True)
        assert page.locator('#mobile-replay-record').input_value()=='3'
        assert 'Arm contacted obstacle' in page.locator('#mobile-status').text_content()
        page.locator('#mobile-filter-outcome').select_option('success')
        assert_ids(page,[])
        assert page.locator('#mobile-filter-empty').is_visible()
        page.locator('#mobile-filter-stage').select_option('4')
        assert_ids(page,[2,4,5])
        choose(page,page.locator('#mobile-diagnostics [data-mobile-index="5"] button').nth(1))
        assert 'All 4 stages completed' in page.locator('#mobile-status').text_content()
        assert page.locator('#mobile-export-data').text_content()==raw.decode('utf-8-sig')
        assert page.locator('#mobile-summary').inner_html()==summary
        english_only(page)
        page.screenshot(path=str(args.output/'successful-filter.png'))
        page.locator('#language-select').select_option('ja')
        assert page.locator('#mobile-filter-outcome').input_value()=='success'
        assert page.locator('#mobile-filter-stage').input_value()=='4'
        page.locator('#mobile-filter-reset').click()
        assert_ids(page,list(range(6)))
        page.wait_for_function("document.querySelector('#mobile-diagnostics').textContent.includes('腕が障害物に接触')")
        page.set_viewport_size({'width':390,'height':844})
        page.locator('#mobile-filter-outcome').select_option('failure')
        page.locator('#mobile-diagnostics [data-mobile-index="1"] button').nth(1).tap()
        page.wait_for_function("document.querySelector('#mobile-replay-record').value==='1'")
        page.screenshot(path=str(args.output/'failure-mobile.png'))
        print('PASS: partial/read-only results, combined filters, empty/reset states, original-index replay, saved pre-end ticks, immutable summary/CSV and ja/en/mobile',flush=True)
        # A quick blocked benchmark exercises five seeds and seed-selection reset.
        page.locator('#mobile-filter-seed').select_option('42')
        page.locator('#experiment-planner-select').select_option('0')
        page.locator('#experiment-seed-editor').fill('43')
        page.locator('#experiment-seed-editor').dispatch_event('change')
        page.locator('#mobile-benchmark-button').tap()
        page.wait_for_function('document.querySelectorAll("#mobile-diagnostics article").length===30',timeout=120000)
        assert page.locator('#mobile-filter-seed').input_value()=='all'
        assert_ids(page,list(range(30)))
        page.locator('#mobile-filter-stage').select_option('0')
        page.locator('#mobile-filter-seed').select_option('45')
        assert_ids(page,list(range(12,18)))
        assert '6/30' in page.locator('#mobile-filter-count').text_content()
        card=page.locator('#mobile-diagnostics [data-mobile-index="17"]')
        choose(page,card.locator('button').nth(0))
        assert page.locator('#mobile-replay-tick').input_value()=='0'
        choose(page,card.locator('button').nth(1))
        assert page.locator('#mobile-replay-record').input_value()=='17'
        _,all_records=download(page,args.output/'all-thirty.csv')
        assert len(all_records)==30 and {r['seed'] for r in all_records}=={'43','44','45','46','47'}
        page.locator('#mobile-stop-button').tap()
        page.wait_for_function('document.querySelectorAll("#mobile-diagnostics button:enabled").length===0')
        assert_ids(page,list(range(12,18)))
        assert not errors,errors
        print('PASS: five-seed filtering, new-batch seed reset, full CSV export, zero-tick short-trial replay and stopped read-only diagnostics',flush=True)
        browser.close()


if __name__=='__main__':
    main()
