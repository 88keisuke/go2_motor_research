#!/usr/bin/env python3
"""Generate auditable synthetic/capture evidence. NEVER imports serial."""
import json
from pathlib import Path
import struct
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from go2driver.control import MotorController, Termination
from go2driver.driver import MotorBus
from go2driver.logging import EventLog, timing_summary
from go2driver.protocol import PROFILES, decode
from go2driver.transport import VirtualTransport


def save(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def main():
    output=ROOT/'validation'
    output.mkdir(exist_ok=True)
    fixtures=json.loads((ROOT/'tests/fixtures/captures.json').read_text())
    crc_report=[]
    capture_events=[]
    for pair in fixtures['frames']:
        for direction in ('tx','rx'):
            frame=bytes.fromhex(pair[direction])
            spec=getattr(PROFILES['stock'],direction+'_crc')
            crc_report.append({'fixture':pair['name'],'direction':direction,
                               'calculated':f'0x{spec.calculate(frame[:-2]):04X}',
                               'stored_little_endian':f'0x{int.from_bytes(frame[-2:],"little"):04X}',
                               'match':spec.calculate(frame[:-2])==int.from_bytes(frame[-2:],'little'),
                               'raw':decode(frame,PROFILES['stock'],direction)})
            capture_events.append({'schema':1,'event':direction,'hex':pair[direction],
                                   'source':'user-supplied past capture; timestamp unavailable',
                                   'fixture':pair['name']})
    (output/'supplied-captures.jsonl').write_text(''.join(json.dumps(e,ensure_ascii=False)+'\n' for e in capture_events),encoding='utf-8')
    official=json.loads((ROOT/'tests/fixtures/official_vectors.json').read_text())
    dev_events=[{'schema':1,'event':'tx','hex':v['hex'],
                 'source':'official SDK synthetic input; no hardware'} for v in official['commands']]
    dev_events.append({'schema':1,'event':'rx','hex':official['feedback']['hex'],
                       'source':'independent synthetic RX decoder vector; not a transaction reply'})
    (output/'official-development.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in dev_events),encoding='utf-8')
    # Independent expected bytes come from captures and the official SDK fixtures.
    # This synthetic responder is only a test plant for control and fault paths.
    def response(request):
        mode=request[2]>>4; motor_id=request[2]&15
        pos=struct.unpack_from('<i',request,7)[0] if mode else 0
        body=b'\xfd\xee'+bytes([request[2]])+struct.pack('<hhibH',0,0,pos,28,0)
        frame=body+PROFILES['stock'].rx_crc.calculate(body).to_bytes(2,'little')
        return [b'noise'+frame[:5],frame[5:]]
    log=EventLog()
    log.emit('simulation',source='synthetic ideal plant; not hardware; host timing only')
    bus=MotorBus(VirtualTransport(responder=response),timeout=0.01,log=log)
    controller=MotorController(bus,termination=Termination(verified=True,evidence='virtual plant only; NOT an actual termination verification'))
    result=controller.run({0:0.001,1:0.002,2:0.003},duration=0.06,period=0.02)
    (output/'virtual-multi.jsonl').write_text(''.join(json.dumps(e,ensure_ascii=False)+'\n' for e in log.events),encoding='utf-8')
    suite=unittest.defaultTestLoader.discover(str(ROOT/'tests'))
    with (output/'unittest.txt').open('w',encoding='utf-8') as handle:
        test_result=unittest.TextTestRunner(stream=handle,verbosity=2).run(suite)
    summary={'hardware_io_performed':False,'tests_run':test_result.testsRun,
             'test_failures':len(test_result.failures),'test_errors':len(test_result.errors),
             'stock_crc':{'normal_polynomial':'0x1021','reflected_polynomial':'0x8408',
                          'bit_order':'LSB first','initial':'0x2CBB','xor_out':'0x0000',
                          'coverage':'TX bytes 0..14; RX bytes 0..13','storage':'little endian',
                          'results':crc_report},
             'single_bit_mutations_checked':sum(len(bytes.fromhex(p[d]))*8 for p in fixtures['frames'] for d in ('tx','rx')),
             'virtual_multi':{'result':result,'timing':timing_summary(log.events)},
             'limits':'matching finite captures cannot establish all protocol behavior or physical units'}
    save(output/'report.json',summary)
    print(json.dumps({'tests_run':test_result.testsRun,'passed':test_result.wasSuccessful(),
                      'capture_crc_matches':sum(r['match'] for r in crc_report),
                      'single_bit_mutations':summary['single_bit_mutations_checked'],
                      'hardware_io_performed':False}))
    if not test_result.wasSuccessful() or not all(r['match'] for r in crc_report):
        raise SystemExit(1)

if __name__=='__main__':
    main()
