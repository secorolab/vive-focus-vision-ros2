#!/usr/bin/env python3
"""Check controller/tool pairing independently of ROS and a headset."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from openarm_align import multiply, pairing, degrees_rpy


def inverse(q):
    return (-q[0], -q[1], -q[2], q[3])


def from_rpy(rpy):
    r,p,y = [math.radians(v)/2 for v in rpy]
    return multiply(multiply((0,0,math.sin(y),math.cos(y)),
                             (0,math.sin(p),0,math.cos(p))),
                    (math.sin(r),0,0,math.cos(r)))


def rotate(q, v):
    return multiply(multiply(q, (*v,0)), inverse(q))[:3]


class Alignment(unittest.TestCase):
    def same_rotation(self, a, b):
        self.assertAlmostEqual(abs(sum(x*y for x,y in zip(a,b))), 1., places=8)

    def test_rpy_round_trip(self):
        for r in (-150,0,45,180):
            for p in (-90,-89,0,40,90):
                for y in (-60,0,120):
                    q=from_rpy((r,p,y))
                    self.same_rotation(q,from_rpy(degrees_rpy(q)))

    def test_tool_pairing(self):
        controller=from_rpy((35,-50,80))
        tool=from_rpy((180,0,0)) # blue tool axis points down
        offset=pairing(controller,tool)
        for move in ((0,0,.05),(.05,0,0),(0,-.05,0)):
            local_controller=rotate(inverse(controller),move)
            local_tool=rotate(inverse(offset),local_controller)
            reconstructed=rotate(tool,local_tool)
            for a,b in zip(move,reconstructed):self.assertAlmostEqual(a,b,places=8)
        for rpy in ((10,0,0),(0,10,0),(0,0,10)):
            world_delta=from_rpy(rpy)
            controller_now=multiply(world_delta,controller)
            delta_controller=multiply(inverse(controller),controller_now)
            delta_tool=multiply(multiply(inverse(offset),delta_controller),offset)
            target=multiply(tool,delta_tool)
            self.same_rotation(target,multiply(world_delta,tool))

    def test_invalid_pose(self):
        for q in ((0,0,0,0),(math.nan,0,0,1)):
            with self.assertRaises(ValueError):pairing(q,(0,0,0,1))


if __name__ == '__main__':
    unittest.main()
