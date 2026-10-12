"""Freeze selected current sources without builds, worlds, Git or network."""
import argparse
from release_regression_common import guard_tools, load, now, ref, save_new, source_files


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tool-freeze-sha', required=True)
    parser.add_argument('--phase', choices=['preparation-only', 'final-restored-default'], required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    guard_tools(args.tool_freeze_sha)
    before = source_files()
    after = source_files()
    assert before == after, 'selected sources changed while freezing'
    save_new(args.output, {'captured_utc': now(), 'phase': args.phase,
        'base_ref_as_reported_by_root': 'c515029a',
        'tool_freeze_sha256': args.tool_freeze_sha,
        'scope': 'Explicit selected rs/toml/lock sources under first-party, examples and third_party plus root configuration; not compiler/dependency closure.',
        'files': before, 'count': len(before), 'native_worlds_executed_by_this_freeze': 0,
        'planned18_coupled_world_count_contribution': 0,
        'planned72_scalar_world_count_contribution': 0})
    print(ref(args.output))


if __name__ == '__main__':
    main()
