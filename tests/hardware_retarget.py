"""Explicit hardware acceptance test; moves the identified stage and homes it.

Run manually with --run-hardware. Never collected by pytest.
"""
import argparse
import time
import serial
from serial.tools import list_ports


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-hardware', action='store_true', required=True)
    args = parser.parse_args()
    port = next(p.device for p in list_ports.comports() if p.serial_number == '20883074534E')
    with serial.Serial(port, 921600, timeout=.15, write_timeout=1) as device:
        def command(text, timeout=4, quiet=False):
            if not quiet:
                print('> ' + text, flush=True)
            device.write((text + '\n').encode('ascii'))
            deadline = time.monotonic() + timeout
            lines = []
            while time.monotonic() < deadline:
                line = device.readline().decode(errors='replace').strip()
                if not line:
                    continue
                if not quiet:
                    print(line, flush=True)
                if line.startswith(('E)', 'error:')) or line == 'busy':
                    raise RuntimeError(line)
                if line == 'ok':
                    return lines
                lines.append(line)
            raise TimeoutError(text)

        def finish():
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline:
                if command('M53', quiet=True) == ['1']:
                    state = command('M57')
                    assert any('drivers=1, fault=0' in line for line in state)
                    return
                time.sleep(.15)
            raise TimeoutError('motion')

        try:
            version = command('M58')
            assert any(v in version for v in ('v1.2.0-stm32-f401', 'v1.3.2-stm32-f401'))
            command('G28', 120)
            for feed in (1, 2):
                for x, y, z in ((0,3.8,0),(0,-3.8,0),(0,0,0),(3.8,0,0),(-3.8,0,0),(0,0,0),(0,0,1),(0,0,-1),(0,0,0)):
                    command(f'G0 X{x} Y{y} Z{z} F{feed}')
                    finish()
            command('G0 X3.8 Y0 Z0 F1')
            time.sleep(.4)
            assert command('M53') == ['0']
            command('G1 X-1 Y1 F1')
            command('M50')
            finish()
            command('G0 X2 Y0 Z0 F1')
            time.sleep(.4)
            command('M0')
            assert command('M53') == ['1']
            command('M57')
            command('G0 X0 Y0 Z0 F1')
            finish()
            print('PASS: speed, retarget, queries during travel, stop/hold', flush=True)
        finally:
            command('M18')


if __name__ == '__main__':
    main()
