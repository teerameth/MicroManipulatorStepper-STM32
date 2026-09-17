"""Opt-in movement, settling and bidirectional position-step recovery test."""
import argparse
import json
import re
import time
import serial
from serial.tools import list_ports


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-hardware', action='store_true', required=True)
    parser.add_argument('--rounds', type=int, default=1)
    parser.add_argument('--skip-home', action='store_true')
    parser.add_argument('--feed', type=float, default=1)
    parser.add_argument('--step-mm', type=float, default=.2)
    args = parser.parse_args()
    port = next(p.device for p in list_ports.comports() if p.serial_number == '20883074534E')
    with serial.Serial(port, 921600, timeout=.1, write_timeout=1) as device:
        def command(text, timeout=4):
            device.write((text+'\n').encode())
            end = time.monotonic()+timeout
            result=[]
            while time.monotonic()<end:
                line=device.readline().decode(errors='replace').strip()
                if not line: continue
                if line.startswith(('E)', 'error:')) or line == 'busy':
                    raise RuntimeError(line)
                if line=='ok': return result
                result.append(line)
                if timeout > 10: print(line, flush=True)
            raise TimeoutError(text)

        def state():
            lines = command('M57')
            if not any('drivers=1, fault=0' in l for l in lines):
                raise RuntimeError('\n'.join(lines))
            errors = [float(x) for l in lines if l.startswith('Joint ') for x in re.findall(r'error=([-+\d.]+) deg', l)]
            assert len(errors)==3
            return errors, lines

        def settled(label, timeout=30):
            end=time.monotonic()+timeout
            within=None
            while time.monotonic()<end:
                errors, lines=state()
                if command('M53')==['1'] and max(map(abs,errors))<.05:
                    within=within or time.monotonic()
                    if time.monotonic()-within>.3:
                        print(json.dumps(dict(test=label, errors=errors, diagnostics=[l for l in lines if 'motion_diag:' in l])),flush=True)
                        return
                else: within=None
                time.sleep(.08)
            raise RuntimeError('did not settle: '+label+'\n'+'\n'.join(lines))

        try:
            print(command('M58'),flush=True)
            if not args.skip_home: command('G28',120)
            else: command('M17')
            settled('home',8)
            for n in range(args.rounds):
                for xyz in ((0,3.8,0),(0,-3.8,0),(0,0,0),(3.8,0,0),(-3.8,0,0),(0,0,0),(0,0,1),(0,0,-1),(0,0,0),(2,2,0),(-2,-2,0),(0,0,0)):
                    cmd=('G0 X%g Y%g Z%g'%xyz)+f' F{args.feed:g}'
                    print(cmd,flush=True);command(cmd);settled(f'round{n} '+cmd)
                for axis in range(3):
                    for sign in (-1,1):
                        xyz=[0.,0.,0.];xyz[axis]=sign*args.step_mm
                        command('G24 X%g Y%g Z%g'%tuple(xyz));settled(f'step axis{axis} sign{sign}',8)
                        command('G24 X0 Y0 Z0');settled(f'recover axis{axis} sign{sign}',8)
            command('G0 X0 Y0 Z0 F1')  # reset diagnostic maxima for holding
            for _ in range(50):
                errors, lines = state()
                assert max(map(abs, errors)) < .05, lines
                time.sleep(.1)
            print(json.dumps(dict(test='five_second_hold', errors=errors,
                diagnostics=[l for l in lines if 'motion_diag:' in l])),flush=True)
            print('PASS: motion, bidirectional step settling, holding',flush=True)
        finally:
            command('M18')


if __name__=='__main__': main()
