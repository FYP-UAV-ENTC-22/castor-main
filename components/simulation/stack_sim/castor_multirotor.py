"""stack_sim's multirotor: Pegasus' Multirotor with the rotor timing of an aircraft, without patching the Pegasus fork.

Two things differ from Pegasus' own classes:

  command to rotor   Pegasus applies the rotor command it already holds and only then asks the backend for the next
                     one, and it reads the sensors after both. With PX4 in lockstep that puts three physics steps
                     between a measurement and the rotor force it causes (12 ms at 250 Hz). Here each step reads the
                     sensors first, then exchanges with PX4, then applies what PX4 answered: one step, which is the
                     lockstep protocol's own (PX4 answers the sensors of step k during step k + 1).
  motor lag          Pegasus' rotors follow their command instantly. LaggedThrustCurve gives them a first-order lag
                     (the vehicle file's propulsion.motor_time_constant).

Import after SimulationApp exists, as everything that needs Pegasus.
"""

import math

from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.vehicles.multirotor import Multirotor
from pegasus.simulator.logic.vehicles.vehicle import Vehicle


class LaggedThrustCurve(QuadraticThrustCurve):
    """QuadraticThrustCurve whose rotor speeds approach their command with time constant `time_constant` [s]."""

    def __init__(self, config, time_constant: float):
        super().__init__(config)
        self.time_constant = float(time_constant)
        self._lagged = [0.0] * self._num_rotors

    def update(self, state, dt: float):
        if self.time_constant > 0.0:
            keep = math.exp(-dt / self.time_constant)
            self._lagged = [keep * w + (1.0 - keep) * float(ref) for w, ref in zip(self._lagged, self._input_reference)]
            self._input_reference = list(self._lagged)
        return super().update(state, dt)


class CastorMultirotor(Multirotor):
    """Multirotor that reads its sensors, exchanges with its backends and applies the rotor forces in that order."""

    def update_sensors(self, dt: float):
        pass  # Pegasus' own physics callback, after update(): the sensors are read in update() instead

    def update(self, dt: float):
        Vehicle.update_sensors(self, dt)  # this step's state, before it goes to PX4
        for backend in self._backends:
            backend.update(dt)

        articulation = self.get_dc_interface().get_articulation(self._stage_prefix)
        if len(self._backends) != 0:
            desired_rotor_velocities = self._backends[0].input_reference()
        else:
            desired_rotor_velocities = [0.0 for i in range(self._thrusters._num_rotors)]
        self._thrusters.set_input_reference(desired_rotor_velocities)
        forces_z, _, rolling_moment = self._thrusters.update(self._state, dt)
        for i in range(4):
            self.apply_force([0.0, 0.0, forces_z[i]], body_part="/rotor" + str(i))
            self.handle_propeller_visual(i, forces_z[i], articulation)
        self.apply_torque([0.0, 0.0, rolling_moment], "/body")
        self.apply_force(self._drag.update(self._state, dt), body_part="/body")
