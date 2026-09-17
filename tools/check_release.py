"""Offline checks only: never imports project modules or starts the simulator."""
from pathlib import Path
import ast
import sys
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
errors = []
scenarios = sorted((root / 'scenarios').glob('S[1-6]_*'))
if len(scenarios) != 6:
    errors.append(f'Expected six scenarios, found {len(scenarios)}')
count = 0
for scenario in scenarios:
    for path in scenario.rglob('*.py'):
        count += 1
        try:
            ast.parse(path.read_text(encoding='utf-8-sig'), filename=str(path))
        except (SyntaxError, UnicodeError) as exc:
            errors.append(str(exc))
    for name in ['main.py', 'wheel_config.ini', 'SUMO/main_sumo_sync.py']:
        if not (scenario / name).is_file():
            errors.append(f'Missing {scenario.name}/{name}')
    config = scenario / 'SUMO/sumo_files/simulation.sumocfg'
    try:
        tree = ET.parse(config)
        for tag in ['net-file', 'route-files']:
            node = tree.find(f'input/{tag}')
            if node is None:
                errors.append(f'{config}: missing {tag}')
                continue
            for filename in node.attrib['value'].split(','):
                if not (config.parent / filename.strip()).is_file():
                    errors.append(f'{config}: missing resource {filename}')
    except (OSError, ET.ParseError) as exc:
        errors.append(str(exc))
print(f'Checked {len(scenarios)} scenarios and {count} Python files.')
for error in errors:
    print('FAIL:', error)
print('NOT TESTED: simulator, hardware, gaze SDK, live SUMO synchronization, Unreal build.')
print('PASS: static packaging checks' if not errors else 'FAIL: static packaging checks')
sys.exit(bool(errors))
