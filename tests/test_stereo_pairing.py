import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'orin_rectified_stream'))
from stereo_pairing import StereoPairer


def frame(ms):return (None,int(ms*1e6),int(ms*1e6),int(ms*1e6))


class PairingTest(unittest.TestCase):
    def test_adjacent_latest_frames_are_not_cross_paired(self):
        p=StereoPairer()
        p.push(0,frame(0));p.push(1,frame(9));p.push(0,frame(33))
        a,b=p.take_latest();self.assertEqual((a[2],b[2]),(0,9000000))
        self.assertIsNone(p.take_latest())
        p.push(1,frame(42));a,b=p.take_latest()
        self.assertEqual((a[2],b[2]),(33000000,42000000))

    def test_reject_over_limit_and_recover(self):
        p=StereoPairer();p.push(0,frame(0));p.push(1,frame(22))
        self.assertIsNone(p.take_latest());self.assertEqual(p.stats['unmatched_drops'],[1,0])
        p.push(0,frame(30));a,b=p.take_latest();self.assertEqual(abs(a[2]-b[2]),8000000)

    def test_overflow_and_missing_camera_do_not_reuse_frames(self):
        p=StereoPairer(capacity=2)
        for ms in [0,33,66]:p.push(0,frame(ms))
        self.assertEqual(p.stats['overflow_drops'],[1,0]);p.push(1,frame(67))
        a,b=p.take_latest();self.assertEqual(a[2],66000000);self.assertIsNone(p.take_latest())

    def test_source_clock_and_arrival_both_required(self):
        p=StereoPairer();p.push(0,(None,0,0,0));p.push(1,(None,16000000,5000000,16000000))
        self.assertIsNone(p.take_latest())
        p=StereoPairer();p.push(0,(None,0,0,0));p.push(1,(None,5000000,16000000,5000000))
        self.assertIsNone(p.take_latest());self.assertEqual(p.stats['arrival_gate_drops'],[1,1])

    def test_boundary_and_no_possible_pair(self):
        p=StereoPairer();p.push(0,frame(0));p.push(1,frame(10));self.assertIsNotNone(p.take_latest())
        p=StereoPairer()
        for ms in [0,33,66]:p.push(0,frame(ms));p.push(1,frame(ms+16))
        self.assertEqual(p.stats['matched_pairs'],0)


if __name__=='__main__':unittest.main()
