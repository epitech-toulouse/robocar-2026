#!/usr/bin/env python3
"""Run duty/USB regressions without ROS, Docker or a connected car."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    vesc = Path(__file__).resolve().parents[1] / 'ros2_ws/src/third_party/vesc'
    compiler = shlex.split(os.environ.get('CXX', 'c++'))
    with tempfile.TemporaryDirectory(prefix='robocar-tests-') as directory:
        for package, test, sources in (
            ('vesc_ackermann', 'duty_cycle', []),
            ('vesc_driver', 'packet_writer', ['src/vesc_packet.cpp', 'src/vesc_packet_factory.cpp']),
        ):
            base = vesc / package
            output = Path(directory) / test
            command = compiler + [
                '-std=c++14', '-Wall', '-Wextra', '-Werror', '-pthread',
                '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-g',
                '-I', str(base / 'include'), str(base / f'test/test_{test}.cpp'),
            ] + [str(base / source) for source in sources] + ['-o', str(output)]
            subprocess.run(command, check=True)
            subprocess.run([str(output)], check=True)


if __name__ == '__main__':
    main()
