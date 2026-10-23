"""Timestamp and transform contracts for bag-based alignment."""
import math
import unittest
from bag_export import interpolate_pose, lookup_pose, compose


class BagAlignmentTests(unittest.TestCase):
    def test_interpolation_wraps_yaw(self):
        p=interpolate_pose([(1.,[0.,0.,math.pi-.1]),(2.,[2.,0.,-math.pi+.1])],1.5,1.1)
        self.assertAlmostEqual(p[0],1.)
        self.assertAlmostEqual(abs(p[2]),math.pi)

    def test_no_extrapolation_and_no_interpolation_across_large_gap(self):
        data=[(1.,[0.,0.,0.]),(2.,[1.,0.,0.])]
        self.assertIsNone(interpolate_pose(data,.9,.2))
        self.assertIsNone(interpolate_pose(data,1.5,.2))
        self.assertIsNone(interpolate_pose(data,2.1,.2))

    def test_static_chain_and_inverse(self):
        edges={('odom','base_footprint'):[(1.,[1.,2.,math.pi/2])],
               ('base_footprint','base_link'):[(0.,[.2,0.,0.])]}
        p=lookup_pose(edges,{('base_footprint','base_link')},'odom','base_link',1.)
        self.assertAlmostEqual(p[0],1.);self.assertAlmostEqual(p[1],2.2)
        inv=lookup_pose(edges,{('base_footprint','base_link')},'base_link','odom',1.)
        for x in compose(p,inv):self.assertAlmostEqual(x,0.)


if __name__=='__main__':unittest.main()
