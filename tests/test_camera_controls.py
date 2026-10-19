import copy
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from capt_head.camera_controls import pipeline_properties, roi_metrics, score_window, validate_profile


class CameraControlsTests(unittest.TestCase):
    def test_manual_restart_reinitializes_argus_and_surfaces_capture_failure(self):
        from orin_rectified_stream.manual_camera import restart_sender
        for failure in (False, True):
            calls = []
            def command(client, cmd):
                calls.append(cmd)
                if cmd.startswith('pgrep'):
                    return '123' if any(c.startswith('nohup') for c in calls) else ''
                if cmd.startswith('cat '):
                    return ('Failed to create CaptureSession' if failure else
                            'Producer has connected\nProducer has connected\n')
                return ''
            with patch('orin_rectified_stream.manual_camera.remote', side_effect=command), \
                 patch('orin_rectified_stream.manual_camera.time.sleep'):
                if failure:
                    with self.assertRaisesRegex(RuntimeError, 'CaptureSession'):
                        restart_sender(MagicMock(), None)
                else:
                    restart_sender(MagicMock(), None)
            reset = calls.index('sudo -n systemctl restart nvargus-daemon')
            launch = next(i for i,c in enumerate(calls) if c.startswith('nohup'))
            self.assertLess(reset, launch)
            self.assertTrue(any('init_demo 3' in c for c in calls[reset:launch]))

    def test_interactive_auto_wb_is_not_a_fixed_white_balance_profile(self):
        from orin_rectified_stream.manual_camera import make_profile
        values = dict(exposure_ms=5., analog_gain=1., digital_gain=1., saturation=1.,
                      tnr_mode=1., tnr_strength=.2, ee_mode=0., ee_strength=0.)
        manual = make_profile(values, 1)
        self.assertEqual(manual['eyes'][0]['exposure_ns'], 5000000)
        self.assertEqual(manual['eyes'][0], manual['eyes'][1])
        values['exposure_ms'] = 11
        with self.assertRaises(ValueError):
            make_profile(values, 1)
        p = self.profile()
        p['mode'] = 'fixed_exposure_auto_wb'
        for eye in p['eyes']:
            eye['wbmode'] = 1
        self.assertIs(validate_profile(p), p)
        p['mode'] = 'fixed'
        with self.assertRaises(ValueError):
            validate_profile(p)

    def profile(self):
        eye = dict(exposure_ns=10000000, analog_gain=2., digital_gain=1., wbmode=3,
                   tnr_mode=0, ee_mode=0, saturation=1., tnr_strength=0., ee_strength=0.)
        return dict(mode="fixed", eyes=[dict(eye), dict(eye)])

    def test_both_eyes_use_explicit_fixed_ranges_and_white_balance(self):
        p = self.profile()
        for i in range(2):
            text = pipeline_properties(p, i)
            self.assertIn('exposuretimerange="10000000 10000000"', text)
            self.assertIn('gainrange="2 2"', text)
            self.assertIn('wbmode=3', text)
            self.assertIn('tnr-mode=0', text)
        for key, value in [('exposure_ns', 40000000), ('analog_gain', float('nan')), ('wbmode', 1)]:
            bad = copy.deepcopy(p)
            bad['eyes'][0][key] = value
            with self.assertRaises(ValueError):
                validate_profile(bad)

    def test_score_requires_visible_unsaturated_windows_and_measures_jitter(self):
        def samples(amplitude):
            result = []
            for i in range(50):
                item = dict(disparity_px=60+amplitude*np.sin(i))
                for e in ('left', 'right'):
                    item[e] = dict(ellipse_rms_px=.15, edge_coverage_deg=320.)
                item['roi'] = [dict(mean_luma=140., clipped_any_channel_fraction=0.)]*2
                result.append(item)
            return result
        good = score_window(samples(.02), 50)
        self.assertTrue(good['eligible'])
        self.assertLess(good['score'], score_window(samples(.3), 50)['score'])
        self.assertFalse(score_window(samples(.02), 100)['eligible'])
        self.assertFalse(score_window([], 50)['eligible'])

    def test_roi_is_ball_interior_and_reports_clipping(self):
        image = np.full((100, 100, 3), [0, 180, 255], np.uint8)
        detection = dict(center=[50., 50.], ellipse=dict(major_px=30., minor_px=30.))
        result = roi_metrics(image, detection)
        self.assertEqual(result['clipped_any_channel_fraction'], 1.)
        self.assertEqual(result['clipped_all_channels_fraction'], 0.)
        self.assertGreater(result['mean_luma'], 100.)


if __name__ == '__main__':
    unittest.main()
