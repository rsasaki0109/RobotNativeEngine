#!/usr/bin/env python3
"""Verify shared experiment settings in real WASM, including custom geometry."""
import argparse
import json
from pathlib import Path
from playwright.sync_api import sync_playwright
from test_g1_mobile_wait_browser import download


def ready(page):
    page.wait_for_function("document.querySelector('#engine-status').dataset.ready==='true'",timeout=90000)


def seed(page, value):
    page.locator('#experiment-seed-editor').fill(str(value))
    page.locator('#experiment-seed-editor').dispatch_event('change')


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
        context=browser.new_context(viewport={'width':1280,'height':900},device_scale_factor=0.5,has_touch=True)
        page=context.new_page()
        errors=[]
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.goto(args.url+'?lang=en')
        ready(page)
        page.locator('#scene-editor').evaluate('e=>e.open=true')
        page.locator('#scene-preset').select_option('narrow')
        page.locator('#scene-preset-apply').click()
        for selector,value in [('#scene-x','1.8'),('#scene-z','1.7'),('#scene-width','0.4'),('#scene-depth','0.3')]:
            page.locator(selector).fill(value)
        page.locator('#scene-tool').select_option('obstacle')
        page.locator('#scene-delayed').check()
        page.locator('#scene-place').click()
        seed(page,137)
        page.locator('#experiment-planner-select').select_option('1')
        page.locator('#experiment-controller-select').select_option('0')
        page.locator('#experiment-recovery-toggle').check()
        page.locator('#manipulation-panel').evaluate('e=>e.open=true')
        page.locator('#arm-task').select_option('2')
        page.locator('#arm-planner').select_option('2')
        page.locator('#arm-controller').select_option('1')
        page.locator('#mobile-panel').evaluate('e=>e.open=true')
        page.locator('#mobile-scenario').select_option('3')
        page.locator('#mobile-recovery-toggle').check()
        page.locator('#share-panel').evaluate('e=>e.open=true')
        page.locator('#share-mode').select_option('nav-compare')
        before=page.locator('#experiment-intent').input_value()
        page.locator('#share-create').click()
        url=page.locator('#share-url').input_value()
        assert '?lang=en#rne=' in url and len(url)<10000
        assert page.locator('#experiment-intent').input_value()==before=='0'
        snapshot=page.evaluate('u=>RNEExperimentShare.decode(new URL(u).hash)',url)
        assert snapshot['seed']==137 and len(snapshot['scene']['obstacles'])==3
        assert snapshot['scene']['obstacles'][-1]['appear_step']==60
        assert snapshot['nav']['task']==3 and snapshot['arm']['controller']==1
        # Clipboard success and fallback preserve the visible selectable link.
        page.evaluate("Object.defineProperty(navigator,'clipboard',{configurable:true,value:{writeText:async value=>window.copiedURL=value}})")
        page.locator('#share-copy').click()
        page.wait_for_function('u=>window.copiedURL===u',arg=url)
        page.evaluate("()=>{navigator.clipboard.writeText=async()=>{throw new Error('denied')};}")
        page.locator('#share-copy').click()
        page.wait_for_function("document.querySelector('#share-status').textContent.includes('manually')")
        assert page.locator('#share-url').input_value()==url
        page.screenshot(path=str(args.output/'share-created.png'))
        # Start in a fresh tab to prove the URL alone carries the configuration.
        receiver=context.new_page()
        receiver.on('pageerror',lambda error:errors.append(str(error)))
        receiver.goto(url)
        ready(receiver)
        assert receiver.locator('html').get_attribute('lang')=='en'
        assert receiver.locator('#experiment-intent').input_value()=='0'
        assert receiver.locator('#experiment-records tr').count()==0
        assert receiver.locator('#experiment-task-select').input_value()=='3'
        assert receiver.locator('#experiment-seed').input_value()=='137'
        assert receiver.locator('#mobile-recovery').input_value()=='1'
        assert receiver.locator('#arm-task').input_value()=='2'
        assert json.loads(receiver.locator('#scene-json').input_value())==snapshot['scene']
        # Changing the form after loading must not alter Run shared conditions.
        seed(receiver,99)
        receiver.locator('#experiment-planner-select').select_option('0')
        receiver.locator('#share-start').focus()
        receiver.keyboard.press('Enter')
        assert receiver.locator('#experiment-seed').input_value()=='137'
        assert receiver.locator('#experiment-planner-select').input_value()=='1'
        receiver.wait_for_function('document.querySelectorAll("#experiment-records tr").length===4',timeout=240000)
        assert all('seed 137' in row for row in receiver.locator('#experiment-records tr').all_text_contents())
        assert json.loads(receiver.locator('#scene-json').input_value())==snapshot['scene']
        print('PASS: custom geometry, scheduled obstacles, explicit start, captured configuration and four real WASM trials',flush=True)
        # Same-document links must load too; impossible arm targets finish quickly.
        arm={**snapshot,'mode':'arm-benchmark','seed':0,'arm':{'task':3,'planner':0,'controller':0}}
        receiver.evaluate('s=>location.hash="rne="+RNEExperimentShare.encode(s)',arm)
        receiver.wait_for_function("document.querySelector('#share-mode').value==='arm-benchmark'")
        receiver.locator('#share-start').click()
        receiver.wait_for_function('document.querySelectorAll("#arm-records tr").length===18',timeout=120000)
        assert '18/18' in receiver.locator('#arm-benchmark').text_content()
        # Delivery benchmark uses the shared scenario/policy and all 30 trials.
        mobile={**snapshot,'mode':'mobile-benchmark','seed':4294967295,'nav':{**snapshot['nav'],'planner':0},'mobile':{'scenario':1,'recovery':True}}
        receiver.evaluate('s=>location.hash="rne="+RNEExperimentShare.encode(s)',mobile)
        receiver.wait_for_function("document.querySelector('#share-mode').value==='mobile-benchmark'")
        receiver.set_viewport_size({'width':390,'height':844})
        receiver.locator('#share-start').tap()
        receiver.wait_for_function('document.querySelectorAll("#mobile-records tr").length===30',timeout=120000)
        _,records=download(receiver,args.output/'shared-delivery.csv')
        assert len(records)==30 and all(r['scenario_id']=='1' and r['recovery']=='true' and r['expected_trials']=='30' and r['completed_stages']=='0' for r in records)
        assert [r['seed'] for r in records]==[str((4294967295+i//6)%4294967296) for i in range(30)]
        receiver.locator('#language-select').select_option('ja')
        changed=receiver.locator('#share-url').input_value()
        assert '?lang=ja#rne=' in changed
        assert receiver.evaluate('u=>RNEExperimentShare.decode(new URL(u).hash)',changed)==mobile
        receiver.screenshot(path=str(args.output/'share-mobile.png'))
        # Invalid same-document data must not mutate settings or recorded evidence.
        existing=receiver.locator('#mobile-export-data').text_content()
        receiver.evaluate("location.hash='rne='+btoa(JSON.stringify({version:999}))")
        receiver.wait_for_function("document.querySelector('#share-status').dataset.error==='true'")
        assert receiver.locator('#experiment-seed').input_value()=='4294967295'
        assert receiver.locator('#mobile-export-data').text_content()==existing
        assert receiver.locator('#share-start').is_disabled()
        assert receiver.locator('#share-copy').is_disabled()
        invalid=context.new_page()
        invalid.goto(args.url+'?lang=en#rne=AAAA')
        invalid.wait_for_function("document.querySelector('#share-status').dataset.error==='true'")
        assert invalid.locator('#experiment-seed').input_value()=='42'
        assert invalid.locator('#share-start').is_disabled()
        assert invalid.locator('#experiment-intent').input_value()=='0'
        assert 'Check the shared link' in invalid.locator('#share-status').text_content()
        assert not errors,errors
        browser.close()
        print('PASS: clipboard fallback, same-document links, arm/delivery benchmarks, mobile, locale and atomic invalid-link handling',flush=True)


if __name__=='__main__':
    main()
