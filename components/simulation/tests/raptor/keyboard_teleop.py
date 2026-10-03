"""Keyboard control for GUI runs of raptor_payload.py: move the formation and fire the release mechanisms.

    I / K       formation +x / -x            arrow Up / Down also work
    J / L       formation +y / -y            arrow Left / Right also work
    U / O       formation up / down          Page Up / Page Down also work
    H           back to the start position
    1 .. 9      release cable 0 .. 8 at the drone end
    Shift+1..9  release cable 0 .. 8 at the payload end

Each press moves the formation target by STEP metres; the target the drones see moves there at no more than SPEED, so
a key press never becomes a step input for RAPTOR. Keys only arrive while Kit renders, i.e. with the window open.
"""

import numpy as np

STEP = 0.25  # m per key press
SPEED = 0.4  # m/s, limit on how fast the formation target moves


class KeyboardTeleop:
    def __init__(self, rig_rt=None, log=print, z_min=-1.0):
        """z_min: lowest formation offset; the runner sets it so the payload can touch down but not sink in."""
        self.rig_rt = rig_rt
        self.z_min = z_min
        self.log = log
        self.goal = np.zeros(3)
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.t = 0.0
        self._sub = None

    # trajectory interface: every RaptorBackend calls this with the same t each physics step
    def offset(self, t):
        dt = t - self.t
        if dt > 0:
            d = self.goal - self.pos
            dist = float(np.linalg.norm(d))
            stepmax = SPEED * dt
            move = d if dist <= stepmax else d / dist * stepmax
            self.vel = move / dt
            self.pos = self.pos + move
            self.t = t
        return self.pos.copy(), self.vel.copy()

    def start(self):
        import carb.input
        import omni.appwindow

        self._ci = carb.input
        keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = keyboard
        self._sub = self._input.subscribe_to_keyboard_events(keyboard, self._on_key)
        K = carb.input.KeyboardInput
        self._moves = {
            K.I: (0, 1), K.UP: (0, 1), K.K: (0, -1), K.DOWN: (0, -1),
            K.J: (1, 1), K.LEFT: (1, 1), K.L: (1, -1), K.RIGHT: (1, -1),
            K.U: (2, 1), K.PAGE_UP: (2, 1), K.O: (2, -1), K.PAGE_DOWN: (2, -1),
        }
        self._digits = {getattr(K, f"KEY_{i}"): i - 1 for i in range(1, 10)}
        self.log("[keys] I/K x, J/L y, U/O z (or arrows, PgUp/PgDn), H home, 1-9 release at the drone end, "
                 "Shift+1-9 at the payload end")

    def stop(self):
        if self._sub is not None:
            self._input.unsubscribe_to_keyboard_events(self._keyboard, self._sub)
            self._sub = None

    def _on_key(self, event, *args):
        if event.type != self._ci.KeyboardEventType.KEY_PRESS:
            return True
        key = event.input
        if key in self._moves:
            axis, sign = self._moves[key]
            self.goal[axis] += sign * STEP
            self.goal[2] = max(self.goal[2], self.z_min)
            self.log(f"[keys] formation goal offset {np.round(self.goal, 2)} m")
        elif key == self._ci.KeyboardInput.H:
            self.goal[:] = 0.0
            self.log("[keys] formation goal back to the start position")
        elif key in self._digits and self.rig_rt is not None:
            cable = self._digits[key]
            end = "payload" if event.modifiers & self._ci.KEYBOARD_MODIFIER_FLAG_SHIFT else "drone"
            try:
                self.rig_rt.request_release(cable, end)
                self.log(f"[keys] release cable {cable} at the {end} end")
            except ValueError as e:
                self.log(f"[keys] {e}")
        return True
