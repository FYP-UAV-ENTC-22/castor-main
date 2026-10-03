"""Stepping that gives the same physics with and without the Isaac Sim window.

world.step(render=True) calls app.update(), which advances a whole rendering frame (1/60 s, about 6.7 steps at
400 Hz), so a step-counting loop ran ~6.7x too long in the GUI. Here every mode takes exactly one PhysX step per
world.step(render=False), and the window is redrawn with world.render(), which runs app.update() with
/app/player/playSimulations off (isaacsim.core.api SimulationContext.render), i.e. without stepping physics.
"""

import time


def run_physics(world, simulation_app, duration, physics_hz, render, render_hz=50.0, realtime=True, on_frame=None,
                before_step=None):
    """Step until world.current_time reaches `duration`. Returns the number of physics steps taken.

    render: redraw every round(physics_hz / render_hz) steps; on_frame() runs just before each redraw (e.g. to move
    visual-only prims). realtime: with render, sleep so simulated time does not run ahead of the wall clock.
    before_step(): runs before every physics step, outside any physics callback; the place for USD changes that
    PhysX must see (changing physics attributes from inside a physics callback crashes omni.physx).
    """
    dt = 1.0 / physics_hz
    every = max(1, int(round(physics_hz / render_hz)))
    t0_wall, t0_sim = time.perf_counter(), world.current_time
    steps = 0
    while world.current_time < duration - 0.5 * dt:
        if not simulation_app.is_running():
            break
        if before_step is not None:
            before_step()
        world.step(render=False)
        steps += 1
        if render and steps % every == 0:
            if on_frame is not None:
                on_frame()
            if realtime:
                ahead = (world.current_time - t0_sim) - (time.perf_counter() - t0_wall)
                if ahead > 0:
                    time.sleep(ahead)
            world.render()
    return steps
