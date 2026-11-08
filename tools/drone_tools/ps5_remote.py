"""Read a PS5 (DualSense) controller on the laptop and send it to remote_link on the drone.

Run through tools/ps5 (it opens the SSH tunnel). One JSON line every 50 ms:
    {"seq": n, "sticks": [forward, left, up], "events": [[id, name], ...]}
left stick = forward/back/left/right, R2 = up, L2 = down (no L1 needed: MANUAL mode, the
controller flies the whole course). Button presses become events with increasing
ids, resent for 2 s (the mission handles each id once). Cross only counts with L1 held.
"""
import argparse
import json
import socket
import sys
import time

DEADZONE = 0.15
RATE_HZ = 20.0
EVENT_KEEP_S = 2.0


def axis(value):
    v = value / 32767.0
    return 0.0 if abs(v) < DEADZONE else max(-1.0, min(1.0, v))


def message(state, seq, events, now):
    """The line to send, from {'lx', 'ly', 'l2', 'r2'} (sticks -1..1, SDL: y down; triggers 0..1)."""
    sticks = [-state['ly'], -state['lx'], state['r2'] - state['l2']]
    recent = [[i, name] for i, name, t in events if now - t <= EVENT_KEEP_S]
    return {'seq': seq, 'sticks': [round(v, 3) for v in sticks], 'events': recent}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=47000)
    ap.add_argument('--dry', action='store_true', help='print instead of sending')
    args = ap.parse_args()
    try:
        import pygame
        from pygame._sdl2 import controller
    except ImportError:
        sys.exit('needs pygame:  python3 -m pip install pygame')
    pygame.init()
    controller.init()
    if controller.get_count() == 0:
        sys.exit('no controller found: pair the PS5 controller (Bluetooth, or USB-C cable) and retry')
    pad = controller.Controller(0)
    print(f'controller: {pad.name}. Left stick move, R2 up, L2 down; L1+Cross land here; Circle hold; Triangle resume; '
          f'Square clear. Ctrl-C to quit.')
    sock = None
    if not args.dry:
        sock = socket.create_connection(('127.0.0.1', args.port), timeout=3.0)
        print(f'connected to the drone (tunnel 127.0.0.1:{args.port})')
    buttons = {'land': pygame.CONTROLLER_BUTTON_A, 'hold': pygame.CONTROLLER_BUTTON_B,
               'resume': pygame.CONTROLLER_BUTTON_Y, 'clear': pygame.CONTROLLER_BUTTON_X}
    was = {name: False for name in buttons}
    events, next_id, seq = [], int(time.time() * 10), 0     # ids rise across restarts of this script
    period = 1.0 / RATE_HZ
    try:
        while True:
            pygame.event.pump()
            now = time.monotonic()
            l1 = bool(pad.get_button(pygame.CONTROLLER_BUTTON_LEFTSHOULDER))
            state = {'l1': l1, 'lx': axis(pad.get_axis(pygame.CONTROLLER_AXIS_LEFTX)),
                     'ly': axis(pad.get_axis(pygame.CONTROLLER_AXIS_LEFTY)),
                     'l2': axis(pad.get_axis(pygame.CONTROLLER_AXIS_TRIGGERLEFT)),
                     'r2': axis(pad.get_axis(pygame.CONTROLLER_AXIS_TRIGGERRIGHT))}
            for name, b in buttons.items():
                down = bool(pad.get_button(b))
                if down and not was[name] and (name != 'land' or l1):
                    next_id += 1
                    events.append((next_id, name, now))
                    print(f'\n>> {name}')
                was[name] = down
            events = [e for e in events if now - e[2] <= EVENT_KEEP_S]
            seq += 1
            msg = message(state, seq, events, now)
            line = json.dumps(msg)
            if sock is None:
                print('\r' + line[:100].ljust(100), end='', flush=True)
            else:
                sock.sendall(line.encode() + b'\n')
                print('\rforward/left/up %s' % msg['sticks'], end='', flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - now)))
    except (KeyboardInterrupt, BrokenPipeError, ConnectionResetError) as exc:
        print(f'\nstopped ({type(exc).__name__}): the drone ignores stale input after 0.3 s')
    finally:
        if sock is not None:
            sock.close()


if __name__ == '__main__':
    main()
