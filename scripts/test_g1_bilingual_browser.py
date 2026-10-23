#!/usr/bin/env python3
"""Check both locales, runtime results, persistence and unchanged simulation data."""
import argparse
from pathlib import Path
from playwright.sync_api import sync_playwright
from test_g1_mobile_wait_browser import download


def english_only(page):
    missing = page.evaluate('''()=>{
      const bad=[]; const ignore='script,style,textarea,[hidden],[data-i18n-ignore],#language-switch';
      const walk=document.createTreeWalker(document.body,NodeFilter.SHOW_TEXT);
      while(walk.nextNode()) {const n=walk.currentNode;if(!n.parentElement.closest(ignore)&&/[\\u3040-\\u30ff\\u3400-\\u9fff]/.test(n.nodeValue))bad.push(n.nodeValue);}
      for(const e of document.querySelectorAll('[aria-label],[title],[placeholder]'))if(!e.closest(ignore))for(const a of ['aria-label','title','placeholder'])if(/[\\u3040-\\u30ff\\u3400-\\u9fff]/.test(e.getAttribute(a)||''))bad.push(e.getAttribute(a));
      return bad;
    }''')
    assert not missing, missing


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
        context = browser.new_context(viewport={'width':1280,'height':900}, device_scale_factor=0.5, has_touch=True)
        page = context.new_page()
        errors=[]
        page.on('pageerror',lambda e: errors.append(str(e)))
        page.goto(args.url+'?lang=en')
        page.wait_for_function("document.querySelector('#engine-status').dataset.ready==='true'",timeout=90000)
        assert page.locator('html').get_attribute('lang')=='en'
        assert page.locator('#engine-status').text_content().startswith('Ready')
        assert page.locator('[data-demo-id]:disabled').count()==0
        english_only(page)
        # Movement and the new foot placement run in the real WASM viewer.
        page.locator('#experiment-panel').evaluate('e=>e.open=false')
        page.locator('#canvas').click(position={'x':600,'y':400})
        page.keyboard.down('KeyW')
        page.wait_for_function("parseFloat(document.querySelector('#engine-position').textContent.split(' ')[1])>=0.24",timeout=60000)
        page.keyboard.up('KeyW')
        page.screenshot(path=str(args.output/'walking.png'))
        page.locator('#control-reset').click()
        page.locator('#experiment-panel').evaluate('e=>e.open=true')
        page.locator('[data-demo-id="nav-detour"]').click()
        assert not page.locator('#demo-gallery').evaluate('e=>e.open')
        # Locale switching must preserve the controls and current run intent.
        page.evaluate("window.originalTaskSelect=document.querySelector('#experiment-task-select')")
        intent=page.locator('#experiment-intent').input_value()
        for lang in ['ja','en','ja','en']:
            page.locator('#language-select').select_option(lang)
        assert page.evaluate("window.originalTaskSelect===document.querySelector('#experiment-task-select')")
        assert page.locator('#experiment-intent').input_value()==intent
        assert page.locator('#experiment-task-select').input_value()=='1'
        page.wait_for_function('document.querySelectorAll("#experiment-records tr").length===4',timeout=180000)
        english_only(page)
        assert 'Planning failed' in page.locator('#experiment-records').text_content()
        assert 'Success' in page.locator('#experiment-records').text_content()
        # Exercise generated delivery summary, options, timeline and immutable CSV.
        page.locator('#mobile-panel').evaluate('e=>e.open=true')
        page.locator('#experiment-planner-select').select_option('0')
        page.locator('#mobile-arm-compare-button').click()
        page.wait_for_function('document.querySelectorAll("#mobile-records tr").length===6',timeout=60000)
        raw, records=download(page,args.output/'canonical.csv')
        assert len(records)==6 and all(r['result']=='計画失敗' for r in records)
        english_only(page)
        assert 'Planning failed' in page.locator('#mobile-records').text_content()
        assert 'Replay failure' in page.locator('#mobile-timeline').text_content()
        page.locator('#language-select').select_option('ja')
        assert '計画失敗' in page.locator('#mobile-records').text_content()
        assert page.locator('#replay-panel summary').text_content()=='リプレイ検査'
        assert page.locator('#mobile-export-data').text_content()==raw.decode('utf-8-sig')
        page.locator('#language-select').select_option('en')
        page.locator('.timeline-terminal[data-mobile-record="5"]').click()
        page.wait_for_function("document.querySelector('#mobile-status').textContent.includes('Replay')")
        assert page.locator('#mobile-replay-record').input_value()=='5'
        assert page.locator('#mobile-export-data').text_content()==raw.decode('utf-8-sig')
        # Imported data is evidence and must never be translated.
        page.locator('#replay-scene').evaluate("e=>e.textContent='成功・配置・日本語のシーン名'")
        page.wait_for_timeout(100)
        assert page.locator('#replay-scene').text_content()=='成功・配置・日本語のシーン名'
        english_only(page)
        page.screenshot(path=str(args.output/'english.png'))
        # Dynamic editor errors must follow the selected locale too.
        page.locator('#scene-editor').evaluate('e=>e.open=true')
        page.locator('#scene-x').fill('99')
        page.locator('#scene-place').click()
        page.wait_for_function("document.querySelector('#scene-editor-status').dataset.error==='true'")
        english_only(page)
        page.locator('#language-select').select_option('ja')
        page.set_viewport_size({'width':390,'height':844})
        page.locator('#language-select').select_option('en')
        assert 'lang=en' in page.url
        page.screenshot(path=str(args.output/'english-mobile.png'))
        # Saved preference without query, and explicit URL overriding that preference.
        page.goto(args.url)
        assert page.locator('html').get_attribute('lang')=='en'
        page.goto(args.url+'?lang=ja')
        assert page.locator('html').get_attribute('lang')=='ja'
        assert not errors,errors
        context.close()
        blocked=browser.new_context()
        blocked.add_init_script("Storage.prototype.getItem=Storage.prototype.setItem=function(){throw new DOMException('blocked','SecurityError')}")
        no_storage=blocked.new_page()
        no_storage.goto(args.url+'?lang=en')
        assert no_storage.locator('html').get_attribute('lang')=='en'
        no_storage.locator('#language-select').select_option('ja')
        assert no_storage.locator('html').get_attribute('lang')=='ja'
        blocked.close()
        browser.close()
        print('PASS: ja/en static and runtime UI, gallery readiness, live switching, immutable CSV/payload, replay, editor errors, mobile, URL/storage precedence and blocked storage',flush=True)


if __name__=='__main__':
    main()
