import contextlib
import io
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from go2driver.calibration import compare_velocity, fit_angle
from go2driver.cli import main
from go2driver.control import Limits, MotorController, SafetyAbort, Termination
from go2driver.driver import BusTimeout, MotorBus
from go2driver.logging import EventLog, compare_logs, replay, timing_summary
from go2driver.model import Command, Units, state_from_frame
from go2driver.protocol import CRCError, FrameError, FrameStream, PROFILES, RawCommand, decode, encode
from go2driver.transport import SerialTransport, VirtualTransport

ROOT = Path(__file__).resolve().parents[1]
CAPTURES = json.loads((ROOT/'tests/fixtures/captures.json').read_text())['frames']
OFFICIAL = json.loads((ROOT/'tests/fixtures/official_vectors.json').read_text())
STOCK, DEV = PROFILES['stock'], PROFILES['development']


def feedback(motor_id=0, mode=0, position=0, speed=0, torque=0, temp=28, error=0):
    body = b'\xfd\xee'+bytes([motor_id | mode<<4])+struct.pack('<hhibH',torque,speed,position,temp,error)
    return body+STOCK.rx_crc.calculate(body).to_bytes(2,'little')


def ideal_response(request):
    # Synthetic ideal plant used only for scheduler/abort tests, not protocol proof.
    motor_id, mode = request[2]&15, request[2]>>4
    pos = struct.unpack_from('<i',request,7)[0] if mode else 0
    return [feedback(motor_id, mode, pos)]


class ProtocolTests(unittest.TestCase):
    def test_all_capture_crc_and_single_bit_corruption(self):
        for pair in CAPTURES:
            for direction in ('tx','rx'):
                frame=bytes.fromhex(pair[direction])
                self.assertTrue(decode(frame,STOCK,direction)['crc_valid'])
                with self.assertRaises(CRCError):
                    decode(frame,DEV,direction)
                for bit in range(len(frame)*8):
                    damaged=bytearray(frame); damaged[bit//8]^=1<<(bit%8)
                    with self.assertRaises(FrameError):
                        decode(bytes(damaged),STOCK,direction)

    def test_independent_official_sdk_vectors(self):
        for v in OFFICIAL['commands']:
            frame=encode(Command(2,*v['values']).raw(Units()),DEV)
            self.assertEqual(frame,bytes.fromhex(v['hex']))
        v=OFFICIAL['feedback']
        state=state_from_frame(bytes.fromhex(v['hex']),DEV,Units(),0)
        for key in ('motor_id','mode','temperature','error'):
            self.assertEqual(getattr(state,key),v[key])
        for key in ('position','velocity','torque'):
            self.assertAlmostEqual(getattr(state,key),v[key],places=6)
        self.assertEqual(state.raw['auxiliary_bits_3_14'],3456)

    def test_capture_fields(self):
        expected=[(0,0,0,7,9272),(1,0,0,1,51509),(2,0,0,10,16706),
                  (0,1,0,-8,9271),(0,1,19,9,9397)]
        for pair, (i,m,t,s,p) in zip(CAPTURES,expected):
            raw=decode(bytes.fromhex(pair['rx']),STOCK)
            self.assertEqual((raw['motor_id'],raw['mode'],raw['torque'],raw['speed'],raw['position']), (i,m,t,s,p))
            self.assertEqual(raw['temperature'],28)
            self.assertEqual(raw['error_candidate'],0)
            self.assertIn('unconfirmed',raw['auxiliary_meaning'])
        raw=decode(feedback(position=-2**31,speed=-32768,torque=-32768,temp=-1),STOCK)
        self.assertEqual((raw['position'],raw['speed'],raw['temperature']),(-2**31,-32768,-1))
        raw=decode(bytes.fromhex(CAPTURES[3]['tx']),STOCK,'tx')
        self.assertEqual((raw['speed'],raw['position'],raw['kp'],raw['kd']),(18,9272,640,64))

    def test_partial_concatenated_noise_and_recovery(self):
        first,second=bytes.fromhex(CAPTURES[0]['rx']),bytes.fromhex(CAPTURES[1]['rx'])
        stream=FrameStream(STOCK)
        self.assertEqual(stream.feed(b'noise\xfd'),[])
        self.assertEqual(stream.feed(first[1:5]),[])
        self.assertEqual(stream.feed(first[5:]+second),[first,second])
        self.assertEqual(stream.noise_bytes,5)
        damaged=bytearray(first); damaged[5]^=1
        self.assertEqual(stream.feed(bytes(damaged)+first),[first])
        self.assertEqual(stream.crc_errors,1)
        self.assertLess(len(stream.buffer),16)

    def test_reject_unsafe_values_and_modes(self):
        for cmd in (RawCommand(15),RawCommand(-1),RawCommand(True),RawCommand(0,2),RawCommand(0,7),
                    RawCommand(0,1,torque=32768),RawCommand(0,1,speed=-32769),
                    RawCommand(0,1,position=2**31),RawCommand(0,1,kp=32768),RawCommand(0,1,kd=-1),
                    RawCommand(0,0,position=1)):
            with self.assertRaises(ValueError): encode(cmd,STOCK)
        for v in (float('nan'),float('inf'),-float('inf'),1e100):
            with self.assertRaises(ValueError): encode(Command(0,position=v).raw(Units()),STOCK)
        for command in (Command(0,kp=25.5999),Command(0,velocity=804.1),Command(0,torque=128)):
            with self.assertRaises(ValueError):command.raw(Units())
        for field in ('position','velocity','torque','kp','kd'):
            with self.assertRaises(ValueError): Command(0,**{field:float('nan')}).raw(Units())

    def test_units_and_angle_inverse(self):
        rotor=state_from_frame(feedback(position=32768,speed=256,torque=256),STOCK,Units(),0)
        output=state_from_frame(feedback(position=32768,speed=256,torque=256),STOCK,Units(axis='output'),0)
        self.assertAlmostEqual(rotor.position,6.2832)
        self.assertAlmostEqual(output.position,6.2832/6.33)
        self.assertEqual(output.torque,6.33)
        self.assertEqual(output.validation['angle'],'uncalibrated')
        self.assertFalse(output.validation['gear_ratio_verified'])
        unit=Units(direction=-1,zero_raw=1000,angle_rad_per_count=0.001,angle_calibration_evidence='fixture')
        self.assertEqual(Command(0,position=0.1).raw(unit).position,900)


class BusTests(unittest.TestCase):
    def test_partial_id_mismatch_and_raw_logging(self):
        rx=bytes.fromhex(CAPTURES[0]['rx'])
        bus=MotorBus(VirtualTransport([[feedback(1),b'noise'+rx[:4],rx[4:]]]),timeout=0.01)
        state=bus.status(0)
        self.assertEqual(state.raw['position'],9272)
        self.assertTrue(any(e['event']=='id_mismatch' for e in bus.log.events))
        self.assertEqual(timing_summary(bus.log.events)['0']['missing_replies'],0)
        self.assertEqual(sum(e['event']=='rx_chunk' for e in bus.log.events),3)

    def test_timeout_and_no_cross_transaction_partial(self):
        rx=feedback()
        bus=MotorBus(VirtualTransport([[rx[:8]],[rx[8:]]]),timeout=0.002)
        for _ in range(2):
            with self.assertRaises(BusTimeout): bus.status(0)
        self.assertEqual(timing_summary(bus.log.events)['0']['missing_replies'],2)

    def test_crc_threshold_and_mode_mismatch(self):
        bad=bytearray(feedback());bad[4]^=1
        bus=MotorBus(VirtualTransport([[bytes(bad)*3+feedback()]]),timeout=0.01)
        with self.assertRaises(CRCError):bus.status(0)
        bus=MotorBus(VirtualTransport([[feedback(mode=1)]]),timeout=0.01)
        with self.assertRaises(FrameError):bus.status(0)

    def test_crc_nonconsecutive_and_resynchronization(self):
        bad=bytearray(feedback());bad[4]^=1
        bus=MotorBus(VirtualTransport([[bytes(bad)+feedback(1)+bytes(bad)+feedback()]]),timeout=0.01,max_consecutive_crc=2)
        self.assertEqual(bus.status(0).motor_id,0)

    def test_transport_errors_and_short_write(self):
        class Short(VirtualTransport):
            def write(self,data):return len(data)-1
        bus=MotorBus(Short(),timeout=0.001)
        with self.assertRaises(OSError):bus.status(0)
        self.assertEqual(timing_summary(bus.log.events)['0']['missing_replies'],1)

    def test_authorized_serial_settings_with_mock_only(self):
        from types import SimpleNamespace
        from unittest.mock import MagicMock
        fake=MagicMock()
        fake.write.return_value=3
        factory=MagicMock(return_value=fake)
        with patch.dict('sys.modules', {'serial':SimpleNamespace(Serial=factory)}):
            transport=SerialTransport('COM7',allow_send=True)
            factory.assert_called_once_with('COM7',baudrate=4000000,bytesize=8,parity='N',stopbits=1,
                                            timeout=0.002,write_timeout=0.05,
                                            xonxoff=False,rtscts=False,dsrdtr=False)
            self.assertEqual(transport.write(b'abc'),3)
            transport.discard_input();transport.close()
            fake.reset_input_buffer.assert_called_once()
            fake.close.assert_called_once()

    def test_serial_guard_before_import(self):
        with self.assertRaises(PermissionError):SerialTransport('not-a-device')


class ControlTests(unittest.TestCase):
    verified=Termination(verified=True,evidence='synthetic virtual test only; not hardware verification')

    def controller(self,responder=ideal_response,limits=None):
        bus=MotorBus(VirtualTransport(responder=responder),timeout=0.002)
        return MotorController(bus,limits=limits,termination=self.verified)

    def test_unverified_termination_and_invalid_targets_no_io(self):
        c=self.controller();c.termination=Termination()
        with self.assertRaises(ValueError):c.run({0:0.001})
        self.assertEqual(c.bus.transport.writes,[])
        c.termination=self.verified
        for target in (float('nan'),10):
            with self.assertRaises(ValueError):c.run({0:target})
        with self.assertRaises(ValueError):c.run({15:0})
        self.assertEqual(c.bus.transport.writes,[])

    def test_single_multi_position_velocity_torque_and_termination(self):
        for kind in ('position','velocity','torque'):
            c=self.controller()
            result=c.run({0:0.001,1:0.002,2:0.003},duration=0.006,period=0.002,kind=kind)
            self.assertTrue(result['trajectory_completed'])
            self.assertFalse(result['mechanical_stop_proven'])
            self.assertEqual([frame[2] for frame in c.bus.transport.writes[:3]],[0,1,2])
            self.assertEqual([frame[2] for frame in c.bus.transport.writes[-3:]],[0,1,2])
            cycles=[e for e in c.bus.log.events if e['event']=='cycle']
            self.assertTrue(cycles)
            self.assertGreater(cycles[0]['skew_s'],0)
            self.assertEqual(set(timing_summary(c.bus.log.events)),{'0','1','2'})
            if kind=='position':
                final=[decode(f,STOCK,'tx') for f in c.bus.transport.writes if f[2]==0x10][-1]
                self.assertEqual(final['position'],Command(0,position=0.001).raw(Units()).position)
                self.assertEqual(final['speed'],0)

    def test_faults_abort_and_attempt_exit_for_all_ids(self):
        for fault in ('timeout','crc','heat','error','following','disconnect'):
            count=0
            def respond(frame):
                nonlocal count
                count+=1
                if frame[2]>>4==1:
                    if fault=='timeout':return []
                    if fault=='disconnect':raise OSError('unplugged')
                    if fault=='crc':
                        damaged=bytearray(feedback(mode=1));damaged[5]^=1
                        return [bytes(damaged)*3]
                    return [feedback(frame[2]&15,1,position=10000 if fault=='following' else 0,
                                     temp=80 if fault=='heat' else 28,error=1 if fault=='error' else 0)]
                return ideal_response(frame)
            c=self.controller(respond)
            with self.assertRaises((TimeoutError,CRCError,SafetyAbort,OSError)):
                c.run({0:0.001,1:0.001},duration=0.01,period=0.002)
            self.assertTrue(any(e['event']=='abort' for e in c.bus.log.events))
            term=[e for e in c.bus.log.events if e['event']=='termination']
            self.assertEqual([e['motor_id'] for e in term],[0,1])
            self.assertTrue(all(e['acknowledged'] for e in term))
            self.assertTrue(all(not e['mechanical_stop_proven'] for e in term))

    def test_missing_exit_reply_never_claims_stop(self):
        count=0
        def respond(frame):
            nonlocal count
            count+=1
            return ideal_response(frame) if count==1 else []
        c=self.controller(respond)
        with self.assertRaises(BusTimeout):c.run({0:0.001},duration=0.01,period=0.002)
        term=[e for e in c.bus.log.events if e['event']=='termination'][0]
        self.assertFalse(term['acknowledged'])
        self.assertFalse(term['mechanical_stop_proven'])

    def test_coarse_platform_monotonic_clock_does_not_erase_timing(self):
        # Windows <=3.12 monotonic may have coarse resolution. All scheduling,
        # deadlines and event timestamps must share the high-resolution clock.
        with patch('time.monotonic', return_value=0):
            c=self.controller()
            c.run({0:0.001,1:0.002},duration=0.004,period=0.002)
        session=next(e for e in c.bus.log.events if e['event']=='session')
        self.assertEqual(session['clock'],'perf_counter')
        cycle=next(e for e in c.bus.log.events if e['event']=='cycle')
        self.assertGreater(cycle['skew_s'],0)

    def test_completion_with_missing_exit_reply_fails(self):
        polls=0
        def respond(frame):
            nonlocal polls
            if frame[2]>>4==0:
                polls+=1
                if polls>1:return []
            return ideal_response(frame)
        c=self.controller(respond)
        with self.assertRaises(SafetyAbort):
            c.run({0:0.001},duration=0.004,period=0.002)
        term=[e for e in c.bus.log.events if e['event']=='termination']
        self.assertFalse(term[0]['acknowledged'])

    def test_start_at_current_angle_and_peak_speed_rejection(self):
        calls=[]
        def respond(frame):
            calls.append(frame)
            if frame[2]>>4==0:return [feedback(position=500)]
            return ideal_response(frame)
        c=self.controller(respond)
        c.run({0:500*6.2832/32768},duration=0.004,period=0.002)
        first=decode(calls[1],STOCK,'tx')
        self.assertLessEqual(abs(first['position']-500),1)
        c=self.controller()
        with self.assertRaises(ValueError):c.run({0:0.5},duration=0.01,period=0.002)
        self.assertTrue(all(f[2]>>4==0 for f in c.bus.transport.writes))


class AnalysisTests(unittest.TestCase):
    def test_calibration_residual_repeatability_and_speed(self):
        samples=json.loads((ROOT/'examples/angle_samples.json').read_text())
        report=fit_angle(samples)
        self.assertAlmostEqual(report['slope_rad_per_count'],0.0001,places=6)
        self.assertGreater(report['rms_residual_rad'],0)
        self.assertEqual(len(report['repeatability']),3)
        self.assertEqual(set(report['leg_slopes_rad_per_count']),{'outbound','return'})
        unit=Units(**report['units'])
        data=[{'time_s':0,'raw':1000,'speed_raw':4},{'time_s':0.1,'raw':1100,'speed_raw':4}]
        speed=compare_velocity(data,unit)['intervals'][0]
        self.assertAlmostEqual(speed['angle_derived_rad_s'],0.1,places=3)
        self.assertGreater(speed['noise_estimate_rad_s'],0)
        self.assertIn('excluded',compare_velocity(data,unit,max_dt=0.05)['intervals'][0])
        with self.assertRaises(ValueError):compare_velocity([data[0],data[0]],unit)
        with self.assertRaises(ValueError):fit_angle([{'raw':1,'angle_rad':0}]*3)
        with self.assertRaises(ValueError):compare_velocity(data,Units())

    def test_jsonl_replay_preserves_raw_chunks_and_config(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'run.jsonl'
            log=EventLog(path)
            bus=MotorBus(VirtualTransport([[feedback()[:7],feedback()[7:]]]),
                         units={0:Units(axis='output')},log=log)
            state=bus.status(0);log.close()
            report=replay(path)
            rx=[r['state'] for r in report['records'] if 'state' in r]
            self.assertEqual(len(rx),1)
            self.assertEqual(rx[0]['raw'],state.raw)
            self.assertEqual(rx[0]['validation']['axis'],'output')
            self.assertEqual(report['timing']['0']['missing_replies'],0)
            wrong=replay(path,'development')
            self.assertTrue(any('error' in r or 'crc_errors' in r for r in wrong['records']))
            with self.assertRaises(FileExistsError):EventLog(path)

    def test_malformed_log_records_report_errors_and_continue(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'bad.jsonl'
            records=['null', '[]', '{"event":"transaction","motor_id":0,"ok":true}',
                     'invalid json', json.dumps({'event':'rx','hex':feedback().hex()})]
            path.write_text('\n'.join(records)+'\n')
            report=replay(path)
            self.assertEqual(sum('error' in r for r in report['records']),4)
            self.assertEqual(sum('state' in r for r in report['records']),1)

    def test_default_cli_never_constructs_serial(self):
        with patch('go2driver.cli.SerialTransport',side_effect=AssertionError('opened serial')):
            for args in (['status','--ids','0'],['move','--id','0','--target','0.01'],
                         ['velocity','--id','0','--target','0.01'],['torque','--id','0','--target','0.01'],
                         ['multi','--targets',str(ROOT/'examples/targets.json')]):
                with contextlib.redirect_stdout(io.StringIO()) as output:main(args)
                self.assertTrue(json.loads(output.getvalue())['dry_run'])
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                main(['move','--id','0','--target','0.01','--send','--device','invalid','--log','invalid'])


if __name__=='__main__':
    unittest.main()
