#!/usr/bin/env python3
"""
teleop.py
- Simple keyboard-based teleop client that connects to the ESP32 WebSocket (/ws) and sends JSON commands.
- Uses websocket-client.

Controls:
  w: forward
  s: backward
  a: left
  d: right
  x: stop
  0-3: select servo id
  r/f: increase/decrease selected servo angle
  q: quit

Usage:
  python teleop.py --host 192.168.1.100 --port 80
"""
import websocket
import json
import argparse
import threading
import time
import sys
import termios, tty

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--host", required=True)
    p.add_argument("--port", type=int, default=80)
    return p.parse_args()

def getch():
    # Unix getch
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def main():
    args = parse_args()
    url = f"ws://{args.host}:{args.port}/ws"
    ws = websocket.create_connection(url)
    print("Connected to", url)
    selected_servo = 0
    servo_angles = [90, 90, 90, 90]
    speed = 120

    print("Controls: w/s/a/d/x - motion, 0-3 select servo, r/f +/- angle, q quit")
    try:
        while True:
            ch = getch()
            if ch == 'q':
                break
            elif ch == 'w':
                msg = {"type":"drive","left":speed,"right":speed}
                ws.send(json.dumps(msg))
            elif ch == 's':
                msg = {"type":"drive","left":-speed,"right":-speed}
                ws.send(json.dumps(msg))
            elif ch == 'a':
                msg = {"type":"drive","left":-int(speed*0.5),"right":int(speed*0.5)}
                ws.send(json.dumps(msg))
            elif ch == 'd':
                msg = {"type":"drive","left":int(speed*0.5),"right":-int(speed*0.5)}
                ws.send(json.dumps(msg))
            elif ch == 'x':
                msg = {"type":"drive","left":0,"right":0}
                ws.send(json.dumps(msg))
            elif ch in ['0','1','2','3']:
                selected_servo = int(ch)
                print("Selected servo", selected_servo)
            elif ch == 'r':
                servo_angles[selected_servo] = min(180, servo_angles[selected_servo] + 5)
                ws.send(json.dumps({"type":"servo","id":selected_servo,"angle":servo_angles[selected_servo]}))
                print("Servo", selected_servo, "angle", servo_angles[selected_servo])
            elif ch == 'f':
                servo_angles[selected_servo] = max(0, servo_angles[selected_servo] - 5)
                ws.send(json.dumps({"type":"servo","id":selected_servo,"angle":servo_angles[selected_servo]}))
                print("Servo", selected_servo, "angle", servo_angles[selected_servo])
            else:
                continue
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        ws.close()

if __name__ == "__main__":
    main()
