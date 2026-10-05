"""Scene pieces shared by the runners here. Import after SimulationApp exists."""

from pxr import Gf, UsdGeom
from scipy.spatial.transform import Rotation

from castor_assets import usd_build as U


class GoalMarker:
    """The goal pose, drawn: a translucent disc the size of the payload with its three axes. No physics."""

    def __init__(self, stage, radius, height, n):
        self.root = UsdGeom.Xform.Define(stage, "/World/goal")
        disc = UsdGeom.Cylinder.Define(stage, "/World/goal/disc")
        disc.CreateRadiusAttr(float(radius))
        disc.CreateHeightAttr(float(height))
        disc.CreateAxisAttr("Z")
        disc.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.85, 0.2)])
        disc.CreateDisplayOpacityAttr([0.35])
        for axis, color in (("X", (0.9, 0.1, 0.1)), ("Y", (0.1, 0.8, 0.1)), ("Z", (0.15, 0.3, 0.95))):
            stick = UsdGeom.Cylinder.Define(stage, f"/World/goal/axis_{axis}")
            stick.CreateRadiusAttr(0.008)
            stick.CreateHeightAttr(0.3)
            stick.CreateAxisAttr(axis)
            stick.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            offset = {"X": (0.15, 0, 0), "Y": (0, 0.15, 0), "Z": (0, 0, 0.15)}[axis]
            UsdGeom.Xformable(stick.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*offset))
        # where each drone's setpoint is, as the policy moves it
        self.setpoints = []
        for i in range(n):
            ball = UsdGeom.Sphere.Define(stage, f"/World/setpoints/drone{i}")
            ball.CreateRadiusAttr(0.035)
            ball.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.75, 0.1)])
            ball.CreateDisplayOpacityAttr([0.7])
            self.setpoints.append(ball.GetPrim())
        self.set_visible(False)

    def set_visible(self, visible):
        for prim in [self.root.GetPrim()] + self.setpoints:
            img = UsdGeom.Imageable(prim)
            img.MakeVisible() if visible else img.MakeInvisible()
        self.visible = visible

    def show(self, goal_pos, goal_rot, setpoints=()):
        """Move the marker to a goal pose (position, rotation matrix) and the balls to the drones' setpoints."""
        if not self.visible:
            self.set_visible(True)
        q = Rotation.from_matrix(goal_rot).as_quat()  # x y z w
        U.set_pose(self.root.GetPrim(), goal_pos, (q[3], q[0], q[1], q[2]))
        for prim, position in zip(self.setpoints, setpoints):
            U.set_pose(prim, position)
