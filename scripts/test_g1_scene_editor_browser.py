#!/usr/bin/env python3
"""Exercise floor picking, custom scenes and saved settings in a real WASM viewer."""
import argparse
import json
from pathlib import Path
from playwright.sync_api import sync_playwright


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',required=True)
    parser.add_argument('--browser',default='/usr/bin/chromium')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    with sync_playwright() as p:
        browser=p.chromium.launch(executable_path=args.browser,headless=True,args=[
            '--disable-dev-shm-usage','--enable-unsafe-webgpu','--enable-unsafe-swiftshader',
            '--use-angle=swiftshader','--enable-features=Vulkan','--use-vulkan=swiftshader'])
        page=browser.new_page(viewport={'width':1280,'height':800},accept_downloads=True,has_touch=True)
        errors=[]
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.on('console',lambda m:errors.append(m.text) if any(s in m.text for s in ['GPUValidationError','Invalid ShaderModule','Error while parsing WGSL']) else None)
        page.goto(args.url)
        page.wait_for_function("document.querySelector('#engine-status').textContent.startsWith('操作できます')",timeout=90000)
        page.locator('#scene-editor > summary').click()
        page.wait_for_function("document.querySelector('#scene-editor-visible').value === '1'")
        page.wait_for_function("document.querySelector('#experiment-status').textContent.includes('シーン編集中')")
        assert page.locator('#experiment-task-select').input_value()=='3'
        # The same floor-picking projection is used after camera orbit and resize.
        before=page.locator('#scene-json').input_value()
        page.locator('#canvas').evaluate("canvas => canvas.addEventListener('pointerup', () => { document.querySelector('#scene-save-local').click(); window.sceneSaveBlocked = document.querySelector('#scene-editor-status').dataset.error === 'true'; }, {once:true})")
        page.locator('#canvas').click(position={'x':750,'y':550})
        page.wait_for_function('(old) => document.querySelector("#scene-json").value !== old',arg=before,timeout=30000)
        assert json.loads(page.locator('#scene-json').input_value())['goal']!=json.loads(before)['goal']
        assert page.locator('#experiment-run-button').is_enabled()
        assert page.evaluate('window.sceneSaveBlocked')
        assert page.evaluate("localStorage.getItem('rne.navigation-scene.v1')") is None
        print('3D floor click updated the custom scene',flush=True)
        # SVG picking uses its CTM rather than its CSS box (letterboxing included).
        page.locator('#scene-editor-map').scroll_into_view_if_needed()
        xy=page.locator('#scene-editor-map').evaluate('svg => {const p=svg.createSVGPoint();p.x=-1.2;p.y=-1.4;const q=p.matrixTransform(svg.getScreenCTM());return {x:q.x,y:q.y};}')
        page.mouse.click(xy['x'],xy['y'])
        assert json.loads(page.locator('#scene-json').input_value())['goal']=={'x_m':-1.2,'z_m':-1.4}
        previous=page.locator('#scene-json').input_value()
        page.locator('#scene-x').fill('');page.locator('#scene-place').click()
        assert page.locator('#scene-json').input_value()==previous
        assert page.locator('#scene-editor-status').get_attribute('data-error')=='true'
        page.locator('#scene-preset').select_option('narrow')
        page.locator('#scene-preset-apply').click()
        narrow=json.loads(page.locator('#scene-json').input_value())
        assert len(narrow['obstacles'])==2
        page.locator('#scene-tool').select_option('obstacle')
        page.locator('#scene-x').fill('-2');page.locator('#scene-z').fill('1.5')
        page.locator('#scene-delayed').check();page.locator('#scene-place').click()
        assert json.loads(page.locator('#scene-json').input_value())['obstacles'][-1]['appear_step']==60
        page.locator('#scene-undo').click()
        assert json.loads(page.locator('#scene-json').input_value())==narrow
        page.locator('#scene-save-local').click()
        page.locator('#scene-clear').click()
        assert not json.loads(page.locator('#scene-json').input_value())['obstacles']
        page.locator('#scene-load-local').click()
        assert json.loads(page.locator('#scene-json').input_value())==narrow
        with page.expect_download() as download:
            page.locator('#scene-export').click()
        saved=json.loads(Path(download.value.path()).read_text())
        assert saved['format']=='rne-navigation-scene' and saved['version']==1
        original=page.locator('#scene-json').input_value()
        bad=json.dumps({**saved,'version':99}).encode()
        page.locator('#scene-import').set_input_files({'name':'bad.json','mimeType':'application/json','buffer':bad})
        page.wait_for_function("document.querySelector('#scene-editor-status').dataset.error === 'true'")
        assert page.locator('#scene-json').input_value()==original
        page.locator('#scene-import').set_input_files({'name':'scene.json','mimeType':'application/json','buffer':json.dumps(saved).encode()})
        page.wait_for_function("document.querySelector('#scene-editor-status').textContent.includes('読み込みました')")
        assert json.loads(page.locator('#scene-json').input_value())==narrow
        assert '実行中' not in page.locator('#experiment-status').inner_text()
        page.locator('#scene-editor-map').scroll_into_view_if_needed()
        page.screenshot(path=str(args.output/'scene-editor-desktop.png'))
        print('SVG edits, timed obstacle, undo, local save/load and versioned file roundtrip passed',flush=True)
        page.locator('#scene-preset').select_option('blocked');page.locator('#scene-preset-apply').click()
        page.locator('#experiment-planner-select').select_option('1')
        page.locator('#experiment-controller-select').select_option('1')
        page.locator('#experiment-run-button').click()
        page.wait_for_function("document.querySelector('#experiment-status').textContent.includes('カスタムシーン') && document.querySelector('#experiment-status').textContent.includes('計画失敗')",timeout=30000)
        assert 'scene ' in page.locator('#experiment-records').inner_text()
        assert page.locator('#experiment-records td').nth(4).inner_text()=='—'
        assert page.locator('#scene-native-status').inner_text()==''
        # A quick edit then run may coalesce before a redraw; latest intent wins.
        page.locator('#scene-editor > summary').click()
        page.evaluate("() => { document.querySelector('#scene-preset').value='open'; document.querySelector('#scene-preset-apply').click(); document.querySelector('#experiment-compare-button').click(); }")
        page.wait_for_function('document.querySelectorAll("#experiment-records tr").length === 4',timeout=240000)
        assert all('成功' in t for t in page.locator('#experiment-records tr').all_inner_texts())
        assert len(set(page.locator('#experiment-records td:first-child').all_inner_texts()))==1
        page.locator('#experiment-manual-button').click()
        page.locator('#scene-editor > summary').click()
        page.set_viewport_size({'width':390,'height':844})
        page.wait_for_function("document.querySelector('#canvas').width === 390")
        page.locator('#scene-editor-map').scroll_into_view_if_needed()
        assert not page.evaluate('document.documentElement.scrollWidth > innerWidth')
        page.locator('#scene-tool').select_option('goal')
        page.locator('#scene-editor-map').scroll_into_view_if_needed()
        xy=page.locator('#scene-editor-map').evaluate('svg => {const p=svg.createSVGPoint();p.x=1.7;p.y=-1.7;const q=p.matrixTransform(svg.getScreenCTM());return {x:q.x,y:q.y};}')
        page.touchscreen.tap(xy['x'],xy['y'])
        assert json.loads(page.locator('#scene-json').input_value())['goal']=={'x_m':1.7,'z_m':-1.7}
        page.screenshot(path=str(args.output/'scene-editor-mobile.png'))
        assert not errors,errors
        browser.close()
    print('Scene editor browser checks passed: 3D/SVG picking, presets, undo, storage/files, invalid imports, custom planning failure, four-pair snapshots and mobile layout')


if __name__=='__main__':
    main()
