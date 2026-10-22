"""Test de la chaîne de sécurité (docs/ROBOCAR_TO_DO_ROS2.md §2), sans manette ni VESC.

Simule /joy (50 Hz) et /drive (50 Hz), et observe /commands/motor/duty_cycle.
Lancement, dans le conteneur où tourne
`ros2 launch robocar_bringup control.launch.py with_joy:=false with_vesc:=false` :
    python3 tests/test_safety_chain.py [--kill-mux]
--kill-mux tue ackermann_mux en dernier test (il faut relancer le launch ensuite).
"""
import math
import subprocess
import sys
import threading
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

LB, RB, A = 4, 5, 0
GAIN = 0.10
MAX_DUTY = 0.10
EPSILON = 1e-6


class Tester(Node):

    def __init__(self):
        super().__init__('safety_tester')
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.create_subscription(Float64, 'commands/motor/duty_cycle', self.on_duty, 50)
        self.duties = []  # (t, value)

    def on_duty(self, msg):
        self.duties.append((time.monotonic(), msg.data))

    def spin_for(self, duration, joy_buttons=None, joy_axes=None, drive=None, steering=0.0, on_start=None):
        """joy_buttons None = pas de /joy ; drive None = pas de /drive."""
        if on_start:
            threading.Thread(target=on_start).start()
        end = time.monotonic() + duration
        next_joy = next_drive = time.monotonic()
        while time.monotonic() < end:
            now = time.monotonic()
            if joy_buttons is not None and now >= next_joy:
                j = Joy()
                j.header.stamp = self.get_clock().now().to_msg()
                j.axes = joy_axes or [0.0] * 6
                j.buttons = [0] * 16
                for b in joy_buttons:
                    j.buttons[b] = 1
                self.joy_pub.publish(j)
                next_joy = now + 0.02
            if drive is not None and now >= next_drive:
                d = AckermannDriveStamped()
                d.header.stamp = self.get_clock().now().to_msg()
                d.drive.speed = drive
                d.drive.steering_angle = steering
                self.drive_pub.publish(d)
                next_drive = now + 0.02
            rclpy.spin_once(self, timeout_sec=0.005)


def window(node, t0, t1):
    return [v for t, v in node.duties if t0 <= t <= t1]


def main():
    rclpy.init()
    n = Tester()
    n.spin_for(3.0)  # découverte DDS
    results = []

    def case(name, expect, **kw):
        n.duties.clear()
        t0 = time.monotonic()
        n.spin_for(1.5, **kw)
        vals = window(n, t0 + 0.5, time.monotonic())
        ok = bool(vals) and all(math.isfinite(v) and abs(v - expect) < EPSILON
                                and abs(v) <= MAX_DUTY + EPSILON for v in vals)
        results.append(ok)
        uniq = sorted(set(round(v, 6) for v in vals))
        print(f"{'OK ' if ok else 'ÉCHEC'} {name}: attendu {expect}, reçu {uniq} ({len(vals)} msgs)")

    def stop_case(name, before, after):
        n.duties.clear()
        n.spin_for(1.0, **before)
        moving = bool(n.duties) and abs(n.duties[-1][1]) > EPSILON
        n.duties.clear()
        t_cut = time.monotonic()
        n.spin_for(1.0, **after)
        # délai jusqu'au premier 0 après la coupure
        zeros = [t for t, v in n.duties if abs(v) < EPSILON]
        tail = [v for t, v in n.duties if t > t_cut + 0.4]
        delay = (zeros[0] - t_cut) if zeros else None
        ok = bool(moving and delay is not None and delay < 0.3 and tail
                  and all(math.isfinite(v) and abs(v) < EPSILON for v in tail))
        results.append(ok)
        d = f'{delay * 1000:.0f} ms' if delay is not None else 'jamais'
        print(f"{'OK ' if ok else 'ÉCHEC'} {name}: arrêt après {d}")

    case('sans manette, /drive nominal 1', 0.0, drive=1.0)
    case('RB maintenu, /drive nominal 1', GAIN, joy_buttons=[RB], drive=1.0)
    case('aucun bouton, /drive nominal 1', 0.0, joy_buttons=[], drive=1.0)
    case('bouton A (pas RB), /drive nominal 1', 0.0, joy_buttons=[A], drive=1.0)
    case('LB + stick 0,5, /drive nominal 1', GAIN * 0.5, joy_buttons=[LB],
         joy_axes=[0.0, 0.5, 0.0, 0.0, 0.0, 0.0], drive=1.0)
    case('LB + RB + stick 0,5, /drive nominal 1', GAIN * 0.5, joy_buttons=[LB, RB],
         joy_axes=[0.0, 0.5, 0.0, 0.0, 0.0, 0.0], drive=1.0)
    case('marche arrière', -0.05, joy_buttons=[RB], drive=-0.5)
    case('plafond avant', MAX_DUTY, joy_buttons=[RB], drive=100.0)
    case('plafond arrière', -MAX_DUTY, joy_buttons=[RB], drive=-100.0)
    case('commande neutre', 0.0, joy_buttons=[RB], drive=0.0)
    case('vitesse NaN', 0.0, joy_buttons=[RB], drive=float('nan'))
    case('direction infinie', 0.0, joy_buttons=[RB], drive=0.5, steering=float('inf'))
    stop_case('RB relâché', dict(joy_buttons=[RB], drive=1.0), dict(joy_buttons=[], drive=1.0))
    stop_case('manette morte (plus de /joy)', dict(joy_buttons=[RB], drive=1.0), dict(drive=1.0))
    stop_case('/drive coupé', dict(joy_buttons=[RB], drive=1.0), dict(joy_buttons=[RB]))

    if '--kill-mux' in sys.argv:
        def kill_mux():
            subprocess.run(['pkill', '-9', '-f', 'ackermann_mux/ackermann_mux'])
        stop_case('mux tué', dict(joy_buttons=[RB], drive=1.0), dict(joy_buttons=[RB], drive=1.0,
                                                                     on_start=kill_mux))

    print(f'{sum(results)}/{len(results)} OK')
    rclpy.shutdown()
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
