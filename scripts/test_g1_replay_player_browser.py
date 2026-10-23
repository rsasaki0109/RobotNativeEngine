#!/usr/bin/env python3
"""Validate playback controls for navigation, arm and delivery saved replays."""
import argparse
from pathlib import Path
from playwright.sync_api import sync_playwright
from test_g1_mobile_diagnostics_browser import choose
from test_g1_bilingual_browser import english_only


def times(page,prefix):
    return page.locator('#'+prefix+'-replay-record').evaluate('e=>e.selectedOptions[0].dataset.times.split(",").map(Number)')


def scrub(page,prefix,index):
    page.locator('#'+prefix+'-replay-frame').evaluate('(e,i)=>{e.value=String(i);e.dispatchEvent(new Event("input",{bubbles:true}));}',index)


def wait_tick(page,prefix,tick):
    status={'nav':'experiment-status','arm':'arm-status','mobile':'mobile-status'}[prefix]
    page.wait_for_function('''a=>{
      const s=document.getElementById(a.status).textContent;
      return (s.includes('Replay')||s.includes('リプレイ'))&&s.includes((a.tick/30).toFixed(2)+' s');
    }''',arg={'status':status,'tick':tick})


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
        for prefix in ['nav','arm','mobile']:
            assert page.locator('#'+prefix+'-replay-play').is_disabled()
            assert page.locator('#'+prefix+'-replay-speed').is_disabled()
        page.locator('#experiment-compare-button').click()
        page.wait_for_function('document.querySelectorAll("#nav-replay-record option").length===4',timeout=180000)
        page.locator('#nav-replay-record').evaluate("e=>e.closest('details').open=true")
        page.locator('#nav-replay-record').select_option('3')
        saved=times(page,'nav')
        records=page.locator('#experiment-records').inner_html()
        assert page.locator('#nav-replay-previous').is_disabled()
        page.locator('#nav-replay-next').click()
        wait_tick(page,'nav',saved[1])
        page.locator('#nav-replay-previous').click()
        wait_tick(page,'nav',saved[0])
        page.locator('#nav-replay-speed').select_option('0.5')
        page.locator('#nav-replay-play').focus()
        page.keyboard.press('Enter')
        page.wait_for_function("Number(document.querySelector('#nav-replay-frame').value)>0",timeout=30000)
        page.locator('#language-select').select_option('ja')
        assert page.locator('#nav-replay-play').get_attribute('aria-pressed')=='true'
        page.locator('#nav-replay-play').click()
        paused=page.locator('#nav-replay-tick').input_value()
        page.wait_for_timeout(800)
        assert page.locator('#nav-replay-tick').input_value()==paused
        assert page.locator('#nav-replay-play').get_attribute('aria-pressed')=='false'
        page.locator('#nav-replay-play').click()
        scrub(page,'nav',min(10,len(saved)-1))
        assert page.locator('#nav-replay-play').get_attribute('aria-pressed')=='false'
        page.locator('#nav-replay-play').click()
        page.locator('#nav-replay-record').select_option('2')
        assert page.locator('#nav-replay-play').get_attribute('aria-pressed')=='false'
        assert page.locator('#nav-replay-frame').input_value()=='0'
        # Hidden tabs pause and cannot consume a long elapsed interval on resume.
        page.locator('#nav-replay-play').click()
        page.evaluate("()=>{Object.defineProperty(document,'hidden',{configurable:true,value:true});document.dispatchEvent(new Event('visibilitychange'));}")
        assert page.locator('#nav-replay-play').get_attribute('aria-pressed')=='false'
        paused=page.locator('#nav-replay-tick').input_value()
        page.wait_for_timeout(500)
        page.evaluate("()=>{Object.defineProperty(document,'hidden',{configurable:true,value:false});document.dispatchEvent(new Event('visibilitychange'));}")
        assert page.locator('#nav-replay-tick').input_value()==paused
        # End clamping, restart-at-end, exit and manual takeover.
        saved=times(page,'nav')
        page.locator('#nav-replay-speed').select_option('4')
        page.locator('#nav-replay-play').click()
        page.wait_for_function("document.querySelector('#nav-replay-play').getAttribute('aria-pressed')==='false' && document.querySelector('#nav-replay-frame').value===document.querySelector('#nav-replay-frame').max",timeout=60000)
        wait_tick(page,'nav',saved[-1])
        assert page.locator('#nav-replay-next').is_disabled()
        restarted=page.locator('#nav-replay-play').evaluate("e=>{e.click();return document.querySelector('#nav-replay-frame').value}")
        assert restarted=='0'
        page.locator('#nav-replay-exit').click()
        page.wait_for_timeout(500)
        assert page.locator('#nav-replay-enabled').input_value()=='0'
        page.locator('#nav-replay-play').click()
        page.locator('#experiment-manual-button').click()
        assert page.locator('#nav-replay-play').get_attribute('aria-pressed')=='false'
        assert page.locator('#nav-replay-enabled').input_value()=='0'
        page.locator('#language-select').select_option('en')
        assert page.locator('#experiment-records').inner_html()==records
        print('PASS: navigation play/pause, keyboard, step, scrub/record changes, speed, locale, hidden tab, end/restart, exit and manual takeover',flush=True)
        # Arm short records still step and end correctly without regenerating results.
        page.locator('#manipulation-panel').evaluate('e=>e.open=true')
        page.locator('#arm-task').select_option('3')
        page.locator('#arm-compare-button').click()
        page.wait_for_function('document.querySelectorAll("#arm-replay-record option").length===6',timeout=120000)
        arm_csv=page.locator('#arm-export-data').text_content()
        page.locator('#arm-replay-speed').select_option('0.5')
        page.locator('#arm-replay-play').click()
        page.wait_for_function("document.querySelector('#arm-replay-play').getAttribute('aria-pressed')==='false'&&document.querySelector('#arm-replay-frame').value===document.querySelector('#arm-replay-frame').max",timeout=30000)
        wait_tick(page,'arm',times(page,'arm')[-1])
        assert page.locator('#arm-export-data').text_content()==arm_csv
        # Delivery failures: start before the collision and replay at half speed.
        page.locator('#demo-gallery').evaluate('e=>e.open=true')
        page.locator('[data-demo-id="delivery-arm"]').click()
        assert page.locator('#mobile-replay-play').is_disabled()
        page.wait_for_function('document.querySelectorAll("#mobile-replay-record option").length===6',timeout=300000)
        raw=page.locator('#mobile-export-data').text_content()
        summary=page.locator('#mobile-summary').inner_html()
        page.locator('#mobile-diagnostic-panel').evaluate('e=>e.open=true')
        page.locator('#mobile-replay-speed').select_option('0.5')
        before=page.locator('#mobile-diagnostics [data-mobile-index="3"] button').nth(0)
        choose(page,before)
        page.locator('#mobile-replay-play').click()
        page.wait_for_function("document.querySelector('#mobile-replay-play').getAttribute('aria-pressed')==='false'&&document.querySelector('#mobile-replay-frame').value===document.querySelector('#mobile-replay-frame').max",timeout=30000)
        wait_tick(page,'mobile',times(page,'mobile')[-1])
        assert 'Arm contacted obstacle' in page.locator('#mobile-status').text_content()
        # A timeline/diagnostic jump must cancel its old animation callback.
        scrub(page,'mobile',0)
        page.locator('#mobile-replay-play').click()
        choose(page,page.locator('#mobile-diagnostics [data-mobile-index="1"] button').nth(1))
        assert page.locator('#mobile-replay-play').get_attribute('aria-pressed')=='false'
        fixed=page.locator('#mobile-replay-tick').input_value()
        page.wait_for_timeout(500)
        assert page.locator('#mobile-replay-tick').input_value()==fixed
        assert page.locator('#mobile-replay-record').input_value()=='1'
        assert page.locator('#mobile-export-data').text_content()==raw
        assert page.locator('#mobile-summary').inner_html()==summary
        page.locator('#language-select').select_option('en')
        english_only(page)
        page.screenshot(path=str(args.output/'replay-player.png'))
        page.set_viewport_size({'width':390,'height':844})
        last=int(page.locator('#mobile-replay-frame').get_attribute('max'))
        page.locator('#mobile-replay-previous').tap()
        assert page.locator('#mobile-replay-frame').input_value()==str(last-1)
        page.locator('#mobile-replay-next').tap()
        assert page.locator('#mobile-replay-frame').input_value()==str(last)
        page.screenshot(path=str(args.output/'replay-player-mobile.png'))
        page.locator('#mobile-replay-play').tap()
        page.locator('#mobile-stop-button').tap()
        page.wait_for_function("document.querySelector('#mobile-replay-play').disabled")
        page.wait_for_timeout(500)
        assert page.locator('#mobile-replay-enabled').input_value()=='0'
        assert page.locator('#mobile-export-data').text_content()==raw
        assert not errors,errors
        browser.close()
        print('PASS: arm/delivery playback, half-speed failure replay, diagnostic jump cancellation, mobile step/stop and immutable evidence',flush=True)


if __name__=='__main__':
    main()
