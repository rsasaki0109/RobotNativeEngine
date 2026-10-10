#!/usr/bin/env python3
"""Compose fleet telemetry around recorded RNE images and encode shareable media."""

from __future__ import annotations

import argparse
import bisect
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import subprocess
import textwrap

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parent
ARTIFACT_ROOT = ROOT.parents[1] / 'target' / 'warehouse-fleet'
ENGINE_INPUTS = (
    'crates/rne_core/Cargo.toml',
    'crates/rne_core/src/rng.rs',
    'crates/rne_core/src/time.rs',
    'crates/rne_nav/Cargo.toml',
    'crates/rne_nav/src/coordination.rs',
    'crates/rne_nav/src/grid.rs',
    'crates/rne_world/Cargo.toml',
    'crates/rne_world/src/resources.rs',
)
FONT_ROOT = Path('/usr/share/fonts/truetype/dejavu')
BG = (8, 17, 26)
PANEL = (15, 28, 40)
BORDER = (34, 53, 67)
TEXT = (229, 239, 244)
MUTED = (140, 162, 177)
TEAL = (45, 207, 167)
BLUE = (83, 176, 245)
AMBER = (251, 186, 76)
PURPLE = (179, 149, 252)
COLORS = {
    'idle': MUTED,
    'to_pick': BLUE,
    'picking': BLUE,
    'to_drop': TEAL,
    'dropping': TEAL,
    'to_charge': PURPLE,
    'charging': PURPLE,
    'waiting': AMBER,
}


@lru_cache(maxsize=32)
def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    filename = 'DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf'
    return ImageFont.truetype(str(FONT_ROOT / filename), size)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition: bool, message: str) -> None:
    """Reject unqualified input even when Python assertions are disabled."""
    if not condition:
        raise ValueError(message)


def verify_source_provenance(trace: dict, source_root: Path = ROOT) -> dict:
    """Bind declared example, workspace and engine inputs, without qualifying binaries."""
    provenance = trace.get('source_provenance')
    require(isinstance(provenance, dict), 'Trace lacks example source provenance')
    require(provenance.get('format_version') == 1, 'Unsupported example source provenance')
    require(provenance.get('simulation_source_sha256') == sha256(source_root / 'main.rs'),
            'Trace example source differs from current main.rs')
    require(provenance.get('package_manifest_sha256') == sha256(source_root / 'Cargo.toml'),
            'Trace example manifest differs from current Cargo.toml')
    workspace = source_root.parents[1]
    require(provenance.get('workspace_manifest_sha256') == sha256(workspace / 'Cargo.toml'),
            'Trace workspace manifest differs from current Cargo.toml')
    require(provenance.get('workspace_lock_sha256') == sha256(workspace / 'Cargo.lock'),
            'Trace workspace lock differs from current Cargo.lock')
    expected = {path: sha256(workspace / path) for path in ENGINE_INPUTS}
    require(provenance.get('engine_sources_sha256') == expected,
            'Trace engine input hashes differ from current declared source inputs')
    return provenance


def validate_trace(trace: dict) -> None:
    verify_source_provenance(trace)
    require(trace['schema_version'] == 1, "Recording integrity check failed: trace['schema_version'] == 1")
    require(trace['robot_count'] == 40, "Recording integrity check failed: trace['robot_count'] == 40")
    validation = trace['validation']
    require(validation['passed'], 'Simulation acceptance gates did not pass')
    require(validation['replay_full_state_bit_exact'], 'Replay did not match')
    require(validation['collision_count'] == 0, 'Swept collision validation failed')
    require(validation['rack_incursion_count'] == 0, 'Rack clearance validation failed')
    require(validation['incident_active_occupancy_violations'] == 0, "Recording integrity check failed: validation['incident_active_occupancy_violations'] == 0")
    required = validation['required_robot_spacing_m']
    require(required >= 1.3, 'Recording integrity check failed: required >= 1.3')
    require(validation['minimum_swept_robot_spacing_m'] >= required - 1e-8, "Recording integrity check failed: validation['minimum_swept_robot_spacing_m'] >= required - 1e-08")
    require(validation['minimum_rack_clearance_m'] >= -1e-8, "Recording integrity check failed: validation['minimum_rack_clearance_m'] >= -1e-08")
    require(validation['completed_tasks'] > 0, "Recording integrity check failed: validation['completed_tasks'] > 0")
    require(validation['completed_charge_visits'] > 0, "Recording integrity check failed: validation['completed_charge_visits'] > 0")
    require(validation['incident_route_changes'] > 0, "Recording integrity check failed: validation['incident_route_changes'] > 0")
    frames = trace['frames']
    require(frames, 'No recorded frames')
    require(any(frame['kpis']['charging'] > 0 for frame in frames), "Recording integrity check failed: any((frame['kpis']['charging'] > 0 for frame in frames))")
    require(any(robot['loaded'] for frame in frames for robot in frame['robots']), "Recording integrity check failed: any((robot['loaded'] for frame in frames for robot in frame['robots']))")
    require(any(frame['incident']['active'] for frame in frames), "Recording integrity check failed: any((frame['incident']['active'] for frame in frames))")
    ids = [robot['id'] for robot in frames[0]['robots']]
    require(len(ids) == len(set(ids)) == trace['robot_count'], "Recording integrity check failed: len(ids) == len(set(ids)) == trace['robot_count']")
    for index, frame in enumerate(frames):
        require([robot['id'] for robot in frame['robots']] == ids, "Recording integrity check failed: [robot['id'] for robot in frame['robots']] == ids")
        if index:
            dt = frame['time_s'] - frames[index - 1]['time_s']
            require(math.isclose(dt, trace['capture_dt_s'], abs_tol=1e-8), "Recording integrity check failed: math.isclose(dt, trace['capture_dt_s'], abs_tol=1e-08)")
        for robot in frame['robots']:
            require(robot['state'] in COLORS, "Recording integrity check failed: robot['state'] in COLORS")
            require(0 <= robot['battery_percent'] <= 100, "Recording integrity check failed: 0 <= robot['battery_percent'] <= 100")
            require(all(math.isfinite(robot[key]) for key in
                       ('x_m', 'z_m', 'yaw_rad', 'speed_m_s', 'battery_percent')), "Recording integrity check failed: all((math.isfinite(robot[key]) for key in ('x_m', 'z_m', 'yaw_rad', 'speed_m_s', 'battery_percent')))")
    fps = trace['playback_speed'] / trace['capture_dt_s']
    require(fps == round(fps), 'Playback rate needs an integral frame rate')


def verify_render(trace_path: Path, raw_dir: Path, indices: list[int]) -> dict:
    trace = json.loads(trace_path.read_text())
    provenance = verify_source_provenance(trace)
    metadata = json.loads((raw_dir / 'render-metadata.json').read_text())
    require(metadata['trace_validated'] is True, "Recording integrity check failed: metadata['trace_validated'] is True")
    require(metadata['diagnostic_only'] is False, "Recording integrity check failed: metadata['diagnostic_only'] is False")
    require(metadata['fixture_only'] is False, "Recording integrity check failed: metadata['fixture_only'] is False")
    require(metadata['trace_sha256'] == sha256(trace_path), 'Rendered trace is stale')
    require(metadata.get('simulation_source_provenance_verified') is True,
            'Renderer did not verify example source provenance')
    require(metadata.get('simulation_source_sha256') == sha256(ROOT / 'main.rs'),
            'Rendered capture example source differs from current main.rs')
    require(metadata.get('source_provenance') == provenance,
            'Rendered capture provenance differs from the qualified trace')
    require(metadata['renderer_source_sha256'] == sha256(ROOT / 'render.rs'), "Recording integrity check failed: metadata['renderer_source_sha256'] == sha256(ROOT / 'render.rs')")
    require(metadata['renderer_manifest_sha256'] == sha256(ROOT / 'Cargo.toml'), "Recording integrity check failed: metadata['renderer_manifest_sha256'] == sha256(ROOT / 'Cargo.toml')")
    require((metadata['width'], metadata['height']) == (1024, 656), "Recording integrity check failed: (metadata['width'], metadata['height']) == (1024, 656)")
    records = {record['index']: record for record in metadata['frames']}
    for index in indices:
        require(index in records, f'Frame {index} lacks capture evidence')
        record = records[index]
        raw = raw_dir / f'frame-{index:04}.png'
        require(record['file'] == raw.name, "Recording integrity check failed: record['file'] == raw.name")
        with Image.open(raw) as image:
            digest = hashlib.sha256(image.convert('RGBA').tobytes()).hexdigest()
        require(digest == record['rgba_sha256'], f'Frame {index} pixels differ from capture')
    return metadata


def label(draw: ImageDraw.ImageDraw, xy: tuple[int, int], value: str,
          size: int = 13, color: tuple = MUTED, bold: bool = False) -> None:
    draw.text(xy, value, font=font(size, bold), fill=color)


def box(draw: ImageDraw.ImageDraw, bounds: tuple[int, int, int, int]) -> None:
    draw.rounded_rectangle(bounds, radius=9, fill=PANEL, outline=BORDER)


def compose(trace: dict, index: int, raw: Path, target: Path) -> None:
    frame = trace['frames'][index]
    scene = Image.open(raw).convert('RGB')
    require(scene.size == (1024, 656), f'Unexpected scene size {scene.size}')
    image = Image.new('RGB', (1280, 800), BG)
    image.paste(scene, (0, 80))
    draw = ImageDraw.Draw(image)
    draw.line((0, 79, 1280, 79), fill=BORDER)
    draw.line((1023, 80, 1023, 736), fill=BORDER)
    label(draw, (20, 11), 'ROBOT NATIVE ENGINE', 17, TEAL, True)
    label(draw, (20, 35), '40-ROBOT WAREHOUSE', 29, TEXT, True)
    label(draw, (725, 17), 'FLEET OPERATIONS', 16, TEXT, True)
    label(draw, (725, 43), 'Dispatch / traffic / charging', 14)
    label(draw, (1042, 15), 'DETERMINISTIC SIMULATION', 12, TEAL, True)
    label(draw, (1042, 41),
          f"t = {frame['time_s']:06.1f} s   |   {trace['playback_speed']:g}x replay", 14, TEXT)

    kpis = frame['kpis']
    cards = [('ENGAGED', kpis['active'], TEAL),
             ('DELIVERED', kpis['completed_tasks'], TEXT),
             ('WAITING', kpis['waiting'], AMBER),
             ('CHARGING', kpis['charging'], PURPLE)]
    for offset, (name, count, color) in enumerate(cards):
        x = 1038 + (offset % 2) * 117
        y = 98 + (offset // 2) * 91
        box(draw, (x, y, x + 107, y + 79))
        label(draw, (x + 11, y + 10), name, 11, MUTED, True)
        label(draw, (x + 10, y + 28), str(count), 32, color, True)

    box(draw, (1038, 286, 1263, 424))
    label(draw, (1050, 296), 'FLEET STATUS', 12, TEXT, True)
    for offset, robot in enumerate(frame['robots']):
        x = 1052 + (offset % 10) * 20
        y = 322 + (offset // 10) * 21
        color = COLORS[robot['state']]
        draw.rounded_rectangle((x, y, x + 13, y + 13), radius=3, fill=color)
    label(draw, (1050, 408), '40 AMRs / recorded state', 10)

    box(draw, (1038, 437, 1263, 541))
    label(draw, (1050, 447), 'COMPLETED TASKS', 12, TEXT, True)
    start = trace['frames'][0]['time_s']
    end = trace['frames'][-1]['time_s']
    initial = trace['frames'][0]['kpis']['completed_tasks']
    ceiling = max(1, trace['frames'][-1]['kpis']['completed_tasks'] - initial)
    points = []
    for item in trace['frames'][:index + 1]:
        x = 1052 + 192 * (item['time_s'] - start) / max(1, end - start)
        y = 523 - 49 * (item['kpis']['completed_tasks'] - initial) / ceiling
        points.append((x, y))
    draw.line((1052, 523, 1244, 523), fill=BORDER)
    if len(points) > 1:
        draw.line(points, fill=TEAL, width=2)

    box(draw, (1038, 553, 1263, 721))
    label(draw, (1050, 563), 'OPERATIONS LOG', 12, TEXT, True)
    events = trace.get('events', [])
    count = bisect.bisect_right([event['time_s'] for event in events], frame['time_s'] + 1e-8)
    y = 586
    for event in events[max(0, count - 3):count]:
        color = AMBER if 'block' in event['kind'].lower() else TEAL
        label(draw, (1050, y), f"{event['time_s']:05.1f}s", 10, color, True)
        message = textwrap.wrap(event['message'], width=27)[:2]
        for line in message:
            label(draw, (1050, y + 15), line, 11, TEXT)
            y += 14
        y += 20

    incident = frame.get('incident', {})
    if incident.get('active'):
        box(draw, (19, 96, 436, 143))
        draw.ellipse((31, 111, 42, 122), fill=AMBER)
        label(draw, (53, 103), 'AISLE CLOSED / ROUTES REPLANNED', 15, AMBER, True)
        label(draw, (53, 124), 'Reservation traffic continues around the closure', 10)

    draw.rectangle((0, 736, 1280, 800), fill=BG)
    draw.line((0, 736, 1280, 736), fill=BORDER)
    x = 20
    for name, color in [('PICKUP', BLUE), ('TRANSPORT', TEAL),
                        ('YIELD / WAIT', AMBER), ('CHARGE', PURPLE)]:
        draw.rounded_rectangle((x, 755, x + 12, 767), radius=3, fill=color)
        label(draw, (x + 22, 751), name, 13, TEXT, True)
        x += 190
    label(draw, (20, 777), 'TASK DISPATCH  /  PICK  /  DELIVER  /  CELL RESERVATIONS  /  CHARGE SCHEDULING', 10)
    label(draw, (963, 756), 'RNE / WAREHOUSE FLEET', 15, TEAL, True)
    target.parent.mkdir(parents=True, exist_ok=True)
    image.save(target, compress_level=3)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', type=Path, default=ARTIFACT_ROOT / 'trace.json')
    parser.add_argument('--raw', type=Path, default=ARTIFACT_ROOT / 'frames-raw')
    parser.add_argument('--output', type=Path, default=ARTIFACT_ROOT)
    parser.add_argument('--preview', type=int)
    parser.add_argument('--compose-only', action='store_true')
    args = parser.parse_args()
    trace = json.loads(args.trace.read_text())
    validate_trace(trace)
    if args.preview is not None:
        verify_render(args.trace, args.raw, [args.preview])
        compose(trace, args.preview, args.raw / f'frame-{args.preview:04}.png',
                args.output / f'preview-{args.preview:04}.png')
        return
    frames_dir = args.output / 'frames'
    frames_dir.mkdir(parents=True, exist_ok=True)
    metadata = verify_render(args.trace, args.raw, list(range(len(trace['frames']))))
    for index in range(len(trace['frames'])):
        compose(trace, index, args.raw / f'frame-{index:04}.png',
                frames_dir / f'frame-{index:04}.png')
    if args.compose_only:
        return
    fps = int(trace['playback_speed'] / trace['capture_dt_s'])
    inputs = ['ffmpeg', '-y', '-v', 'warning', '-filter_complex_threads', '2',
              '-filter_threads', '2', '-threads', '4', '-framerate', str(fps),
              '-i', str(frames_dir / 'frame-%04d.png')]
    gif = args.output / 'warehouse-fleet.gif'
    subprocess.run(inputs + ['-filter_complex',
        '[0:v]split[a][b];[a]palettegen=stats_mode=diff:max_colors=192[p];'
        '[b][p]paletteuse=dither=bayer:bayer_scale=3:diff_mode=rectangle',
        '-loop', '0', str(gif)], check=True)
    mp4 = args.output / 'warehouse-fleet.mp4'
    subprocess.run(inputs + ['-c:v', 'libx264', '-threads', '4', '-preset', 'slow', '-crf', '20',
                            '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(mp4)], check=True)
    with Image.open(gif) as decoded:
        require(decoded.n_frames == len(trace['frames']), "Recording integrity check failed: decoded.n_frames == len(trace['frames'])")
        durations = []
        for index in range(decoded.n_frames):
            decoded.seek(index)
            durations.append(decoded.info['duration'])
        require(set(durations) == {1000 // fps}, 'Recording integrity check failed: set(durations) == {1000 // fps}')
        dimensions = decoded.size
    poster_index = min(80, len(trace['frames']) - 1)
    Image.open(frames_dir / f'frame-{poster_index:04}.png').save(args.output / 'warehouse-fleet.png')
    proof = {'schema_version': 1, 'trace_sha256': sha256(args.trace),
             'encoder_sha256': sha256(Path(__file__)), 'robot_count': trace['robot_count'],
             'render_metadata_sha256': sha256(args.raw / 'render-metadata.json'),
             'renderer_source_sha256': metadata['renderer_source_sha256'],
             'simulation_source_provenance': trace['source_provenance'],
             'frame_count': len(durations), 'dimensions_px': dimensions,
             'playback_fps': fps, 'playback_speed': trace['playback_speed'],
             'duration_s': sum(durations) / 1000,
             'gif': {'sha256': sha256(gif), 'bytes': gif.stat().st_size},
             'mp4': {'sha256': sha256(mp4), 'bytes': mp4.stat().st_size},
             'source': 'Actual recorded deterministic RNE kinematic fleet simulation; no generated motion.',
             'simulation_validation': trace.get('validation', {})}
    (args.output / 'media-proof.json').write_text(json.dumps(proof, indent=2) + '\n')
    print(json.dumps(proof, indent=2))


if __name__ == '__main__':
    main()
