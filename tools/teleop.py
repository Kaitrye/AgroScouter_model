#!/usr/bin/env python3
"""Keyboard control with a heartbeat for the robot's command relay."""

import codecs
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


KEYS = {
    'i': (1, 0),
    'ш': (1, 0),
    'u': (1, 1),
    'г': (1, 1),
    'o': (1, -1),
    'щ': (1, -1),
    'j': (0, 1),
    'о': (0, 1),
    'l': (0, -1),
    'д': (0, -1),
    ',': (-1, 0),
    'б': (-1, 0),
    'm': (-1, -1),
    'ь': (-1, -1),
    '.': (-1, 1),
    'ю': (-1, 1),
}


def main():
    if not sys.stdin.isatty():
        raise SystemExit('Запусти teleop.sh в отдельном интерактивном терминале.')

    rclpy.init()
    node = Node('robot_keyboard_teleop')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    speed = 0.15
    turn = 0.3
    key_timeout = 1.0
    interval = 0.1
    old_settings = termios.tcgetattr(sys.stdin)
    linear = angular = 0
    last_key = None
    decoder = codecs.getincrementaldecoder('utf-8')(errors='ignore')

    print('Управление: i вперёд, , назад, j/l поворот, u/o/m/. дуга, k или пробел стоп, Ctrl+C выход.')
    print('Работают английская и русская раскладки. Держи клавишу движения нажатой;')
    print('команда обнуляется через 1 секунду без ввода.')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], interval)
            if ready:
                for key in decoder.decode(os.read(sys.stdin.fileno(), 64)):
                    if key == '\x03':
                        return
                    linear, angular = KEYS.get(key, (0, 0))
                    last_key = time.monotonic()
            if last_key is None or time.monotonic() - last_key > key_timeout:
                linear = angular = 0
            command = Twist()
            command.linear.x = linear * speed
            command.angular.z = angular * turn
            publisher.publish(command)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
