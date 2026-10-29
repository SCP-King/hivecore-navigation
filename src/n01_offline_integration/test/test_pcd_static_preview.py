"""G3.2 bounded static PCD preview is read-only and never creates a planner."""
import pathlib
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from n01_offline_integration.pcd_static_preview import inspect_pcd, sample_pcd


HEADER = (b"# .PCD v0.7\nVERSION .7\n"
          b"FIELDS intensity x y z\nSIZE 4 4 4 4\n"
          b"TYPE F F F F\nCOUNT 1 1 1 1\n"
          b"WIDTH 2\nHEIGHT 1\nPOINTS 2\n")


class StaticPcdPreviewTests(unittest.TestCase):
    def test_binary_xyz_is_read_without_editing_pcd(self):
        with tempfile.TemporaryDirectory() as d:
            file = pathlib.Path(d) / "map.pcd"
            raw = HEADER + b"DATA binary\n" + struct.pack(
                "<ffffffff", 4., 1., 2., 3., 9., 5., 6., 7.)
            file.write_bytes(raw)
            info = inspect_pcd(file)
            self.assertEqual(info["count"], 2)
            xyz, meta = sample_pcd(file, 1000)
            self.assertEqual(xyz, [(1., 2., 3.), (5., 6., 7.)])
            self.assertEqual(meta["bounds_xyz_m"], [[1., 5.], [2., 6.], [3., 7.]])
            self.assertEqual(file.read_bytes(), raw)

    def test_ascii_xyz_with_extra_field(self):
        with tempfile.TemporaryDirectory() as d:
            file = pathlib.Path(d) / "factory.pcd"
            file.write_bytes(HEADER + b"DATA ascii\n"
                             + b"1.0 11 12 13\n2.0 21 22 23\n")
            points, meta = sample_pcd(file, 1000)
            self.assertEqual(points, [(11., 12., 13.), (21., 22., 23.)])
            self.assertEqual(meta["format"], "ascii")

    def test_rejects_unsupported_compressed_map(self):
        with tempfile.TemporaryDirectory() as d:
            file = pathlib.Path(d) / "compressed.pcd"
            file.write_bytes(HEADER + b"DATA binary_compressed\n")
            with self.assertRaisesRegex(ValueError, "not supported"):
                inspect_pcd(file)

    def test_rejects_truncation_and_bad_geometry(self):
        with tempfile.TemporaryDirectory() as d:
            file = pathlib.Path(d) / "bad.pcd"
            file.write_bytes(HEADER + b"DATA binary\n" + struct.pack("<ffff",1,2,3,4))
            with self.assertRaisesRegex(ValueError,"truncated"):
                sample_pcd(file, 1000)
            file.write_bytes(HEADER.replace(b"POINTS 2", b"POINTS 3")
                             + b"DATA ascii\n0 1 2 3\n0 4 5 6\n")
            with self.assertRaises(ValueError):
                inspect_pcd(file)

    def test_nonfinite_sample_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            file = pathlib.Path(d) / "nan.pcd"
            file.write_bytes(HEADER + b"DATA ascii\n"
                             + b"0 nan 2 3\n0 1 2 3\n")
            points, meta = sample_pcd(file,1000)
            self.assertEqual(points, [(1.,2.,3.)])
            self.assertEqual(meta["sampled_points"],1)

    def test_downsample_bounded(self):
        with tempfile.TemporaryDirectory() as d:
            file=pathlib.Path(d)/"large.pcd"
            header=(b"FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                    b"WIDTH 1003\nHEIGHT 1\nPOINTS 1003\nDATA binary\n")
            file.write_bytes(header + struct.pack("<fff",1,2,3)*1003)
            points, meta=sample_pcd(file,1000)
            self.assertEqual(len(points),502)
            self.assertEqual(meta["sample_stride"],2)


if __name__ == "__main__":
    unittest.main()
