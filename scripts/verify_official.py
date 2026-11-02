#!/usr/bin/env python3
"""Compile an offline-only harness against pinned official SDK; compare fixtures.

Requires Linux x86_64 and g++; Python driver/tests themselves are cross-platform.
Does not compile or invoke SerialPort or vendor motor examples.
"""
import json
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
build = root/'build'
build.mkdir(exist_ok=True)
sdk = root/'references/unitree_actuator_sdk'
subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-I',str(sdk/'include'),
                str(root/'scripts/official_vectors.cpp'),'-L',str(sdk/'lib'),
                '-lUnitreeMotorSDK_Linux64',f'-Wl,-rpath,{sdk / "lib"}',
                '-o',str(build/'official_vectors')],check=True)
output = subprocess.check_output([str(build/'official_vectors')],text=True)
fixture = json.loads((root/'tests/fixtures/official_vectors.json').read_text())
lines = output.splitlines()
assert lines[0] == 'gear=6.33'
assert lines[1:5] == [v['hex'] for v in fixture['commands']]
assert lines[6] == fixture['feedback']['hex']
fields = dict(v.split('=') for v in lines[5].split(','))
expected = fixture['feedback']
assert fields['rx'] == '1'
for key, name in [('id','motor_id'),('mode','mode'),('temp','temperature'),('error','error'),('aux_sdk','auxiliary_bits_3_14')]:
    assert int(fields[key]) == expected[name]
for key,name in [('q','position'),('dq','velocity'),('tau','torque')]:
    assert abs(float(fields[key])-expected[name]) < 1e-7
print(output,end='')
print('PASS: official SDK independent TX (4) / RX (1) vectors. No serial I/O.')
