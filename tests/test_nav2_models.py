import math, unittest
from app.navigation.nav2_models import *
class Nav2ModelsTest(unittest.TestCase):
    def test_pose_roundtrip(self):
        p=Nav2Pose(x=1,y=2,yaw_rad=1.2,provenance={"type":"test"})
        q=Nav2Pose.from_dict(p.to_dict())
        self.assertAlmostEqual(q.yaw_rad,1.2); self.assertAlmostEqual(quaternion_to_yaw(p.quaternion),1.2)
    def test_nonfinite_rejected(self):
        with self.assertRaises(ValueError): Nav2Pose(x=math.nan).validate()
    def test_execute_double_gate(self):
        req=Nav2Request("id",Nav2Mode.EXECUTE,Nav2Pose(),"now")
        with self.assertRaisesRegex(ValueError,"NOT_ALLOWED"): req.validate()
    def test_terminal(self): self.assertTrue(Nav2JobState.SUCCEEDED.terminal)
