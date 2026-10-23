#!/usr/bin/env python3
"""Real WASM checks for navigation recovery, synchronized replay and G1 manipulation."""
import argparse
from pathlib import Path
from playwright.sync_api import sync_playwright


def seek(page, prefix, fraction):
    page.locator(f'#{prefix}-replay-frame').evaluate('''(el, fraction) => {
        el.value=String(Math.round(Number(el.max)*fraction));
        el.dispatchEvent(new Event('input',{bubbles:true}));
    }''', fraction)
    page.wait_for_function('''prefix => {
      const status=document.querySelector(prefix==='nav'?'#experiment-status':'#arm-status').textContent;
      return status.includes('リプレイ') && status.includes(document.querySelector('#'+prefix+'-replay-time').value);
    }''', arg=prefix)


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
        page=browser.new_page(viewport={'width':1280,'height':900},has_touch=True)
        errors=[]
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.on('console',lambda m:errors.append(m.text) if any(x in m.text for x in ['GPUValidationError','Invalid ShaderModule','Error while parsing WGSL']) else None)
        page.goto(args.url)
        page.wait_for_function("document.querySelector('#engine-status').textContent.startsWith('操作できます')",timeout=90000)
        page.locator('#experiment-task-select').select_option('5')
        page.locator('#experiment-recovery-toggle').check()
        page.locator('#experiment-compare-button').click()
        page.wait_for_function('document.querySelectorAll("#experiment-records tr").length===4',timeout=240000)
        rows=page.locator('#experiment-records tr').all_text_contents()
        assert all('待機あり' in row for row in rows), rows
        assert any('成功' in row and '待機 0.00' not in row for row in rows), rows
        page.locator('#nav-replay-record').evaluate("e=>e.closest('details').open=true")
        assert page.locator('#nav-replay-record option').count()==4
        page.locator('#nav-replay-record').select_option('0')
        seek(page,'nav',0)
        first=page.locator('#experiment-map').inner_html()
        seek(page,'nav',1)
        assert page.locator('#experiment-map').inner_html()!=first
        frozen=page.locator('#experiment-map').inner_html()
        page.evaluate('()=>new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))')
        assert page.locator('#experiment-map').inner_html()==frozen
        assert page.locator('#experiment-records tr').all_text_contents()==rows
        page.screenshot(path=str(args.output/'navigation-replay.png'))
        page.locator('#nav-replay-exit').click()
        page.wait_for_function("!document.querySelector('#experiment-status').textContent.includes('リプレイ')")
        print('Recovery and synchronized navigation replay preserve measured results',flush=True)
        page.locator('#manipulation-panel').evaluate('e=>e.open=true')
        page.locator('#arm-task').select_option('1')
        page.locator('#arm-planner').select_option('1')
        page.locator('#arm-controller').select_option('1')
        page.locator('#arm-run-button').click()
        page.wait_for_function("document.querySelector('#arm-status').textContent.includes('実行中')")
        page.locator('#arm-pause-button').click()
        page.wait_for_function("document.querySelector('#arm-pause-button').getAttribute('aria-pressed')==='true'")
        frozen=page.locator('#arm-status').inner_text()
        page.evaluate('()=>new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))')
        assert page.locator('#arm-status').inner_text()==frozen
        page.locator('#arm-step-button').click()
        page.wait_for_function('(old)=>document.querySelector("#arm-status").textContent!==old',arg=frozen)
        page.locator('#arm-compare-button').click()
        page.wait_for_function('document.querySelectorAll("#arm-records tr").length===6',timeout=240000)
        arm_rows=page.locator('#arm-records tr').all_text_contents()
        assert all('成功' in row for row in arm_rows),arm_rows
        assert page.locator('#arm-replay-record option').count()==6
        page.locator('#arm-replay-record').select_option('2')
        seek(page,'arm',0)
        assert '未保持' in page.locator('#arm-status').inner_text()
        held=False
        for fraction in [0.25,0.5,0.75]:
            seek(page,'arm',fraction)
            if '物体保持中' in page.locator('#arm-status').inner_text():
                held=True
                break
        assert held
        page.screenshot(path=str(args.output/'arm-holding-replay.png'))
        seek(page,'arm',1)
        assert '未保持' in page.locator('#arm-status').inner_text()
        assert page.locator('#arm-records tr').all_text_contents()==arm_rows
        page.locator('#arm-replay-exit').click()
        print('Real G1 arm comparison, pause/step, grasp/release and replay passed',flush=True)
        page.locator('#arm-task').select_option('2')
        page.locator('#arm-planner').select_option('2')
        page.locator('#arm-controller').select_option('1')
        page.locator('#arm-run-button').click()
        page.wait_for_function("document.querySelector('#arm-status').textContent.includes('RRT-Connect') && document.querySelector('#arm-status').textContent.includes('障害物越しに配置') && document.querySelector('#arm-status').textContent.includes('成功') && document.querySelectorAll('#arm-records tr').length===1",timeout=240000)
        assert page.locator('#arm-records tr').count()==1
        page.locator('#arm-compare-button').click()
        page.wait_for_function('document.querySelectorAll("#arm-records tr").length===6',timeout=240000)
        blocked=page.locator('#arm-records tr').all_text_contents()
        assert all('接触' in row for row in blocked[:4]),blocked
        assert all('RRT-Connect' in row and '成功' in row for row in blocked[4:]),blocked
        page.locator('#arm-replay-record').select_option('4')
        seek(page,'arm',0.5)
        page.screenshot(path=str(args.output/'rrt-obstacle-replay.png'))
        page.locator('#arm-task').select_option('3')
        page.locator('#arm-compare-button').click()
        page.wait_for_function("document.querySelectorAll('#arm-records tr').length===6 && [...document.querySelectorAll('#arm-records tr')].every(r=>r.textContent.includes('IK失敗'))",timeout=240000)
        print('RRT-Connect selection, blocked six-pair comparison, saved plans and unreachable failures passed',flush=True)
        page.locator('#experiment-manual-button').click()
        page.wait_for_function("document.querySelector('#playground').dataset.experimentActive==='false'")
        page.set_viewport_size({'width':390,'height':844})
        page.wait_for_function("document.querySelector('#canvas').width===390")
        page.locator('#arm-task').select_option('0')
        page.locator('#arm-run-button').tap()
        page.wait_for_function("document.querySelector('#playground').dataset.experimentActive==='true'")
        assert not page.evaluate('document.documentElement.scrollWidth>innerWidth')
        page.screenshot(path=str(args.output/'manipulation-mobile.png'))
        assert not errors,errors
        browser.close()
    print('Recovery/manipulation browser checks passed, including failure cases and mobile touch')


if __name__=='__main__':
    main()
