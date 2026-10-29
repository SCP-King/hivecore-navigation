import math
import pathlib
import struct
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from n01_offline_integration.core import (
    ReadinessGate, lookup_path, quaternion_rotate, xyz_transform_bytes,
)


class CoreSafetyTests(unittest.TestCase):
    def test_missing_status_denied(self):
        gate = ReadinessGate()
        self.assertFalse(gate.check(now=10)[0])

    def test_explicit_nested_status_contract(self):
        gate = ReadinessGate(outer_path="state.outer", inner_path="state.inner",
                             tf_flag_path="registration_output_tf_published")
        msg = SimpleNamespace(state=SimpleNamespace(outer="Localization", inner="tracking"),
                              registration_output_tf_published=True)
        self.assertTrue(gate.observe(msg, received_monotonic=4))
        self.assertTrue(gate.check(now=4.1)[0])
        self.assertFalse(gate.check(now=5.2)[0])
        msg.registration_output_tf_published = False
        self.assertFalse(gate.observe(msg, received_monotonic=6))
        self.assertFalse(gate.check(now=6.1)[0])

    def test_unverified_fields_denied(self):
        gate = ReadinessGate(outer_path="wrong", inner_path="inner")
        msg = SimpleNamespace(registration_output_tf_published=True)
        self.assertFalse(gate.observe(msg, received_monotonic=1))
        self.assertIn("contract_mismatch", gate.check(now=1)[1])

    def test_fixture_cannot_imply_real_status(self):
        fixture = ReadinessGate(fixture=True)
        self.assertTrue(fixture.observe(SimpleNamespace(data=True), received_monotonic=1))
        strict = ReadinessGate()
        self.assertFalse(strict.observe(SimpleNamespace(data=True), received_monotonic=1))

    def test_quaternion(self):
        v = quaternion_rotate((0,0,math.sqrt(.5),math.sqrt(.5)), (1,0,0))
        self.assertAlmostEqual(v[0],0,places=6)
        self.assertAlmostEqual(v[1],1,places=6)
        with self.assertRaises(ValueError):
            quaternion_rotate((0,0,0,0), (0,0,0))

    def test_transform_preserves_extra_field_and_padding(self):
        # One point, with intensity FLOAT32 and 4 bytes padding after row.
        data=struct.pack("<ffff",1.,2.,3.,9.)+b"\0\0\0\0"
        fields=[("x",0,7,1),("y",4,7,1),("z",8,7,1),("intensity",12,7,1)]
        result=xyz_transform_bytes(data,1,1,16,20,fields,False,(10,0,-1),(0,0,0,1))
        self.assertEqual(struct.unpack("<ffff", result[:16]),(11.,2.,2.,9.))
        self.assertEqual(result[16:],data[16:])

    def test_bigendian_cloud(self):
        data=struct.pack(">fff",1.,2.,3.)
        fields=[("x",0,7,1),("y",4,7,1),("z",8,7,1)]
        result=xyz_transform_bytes(data,1,1,12,12,fields,True,(0,0,.5),(0,0,0,1))
        self.assertEqual(struct.unpack(">fff",result),(1.,2.,3.5))

    def test_reject_corrupt_cloud(self):
        fields=[("x",0,7,1),("y",4,7,1),("z",8,7,1)]
        with self.assertRaises(ValueError):
            xyz_transform_bytes(b"\0"*4,2,1,12,24,fields,False,(0,0,0),(0,0,0,1))
        with self.assertRaises(ValueError):
            xyz_transform_bytes(b"\0"*12,1,1,12,12,
                                [("x",0,7,1)],False,(0,0,0),(0,0,0,1))


if __name__ == "__main__":
    unittest.main()
