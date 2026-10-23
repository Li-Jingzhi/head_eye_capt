import importlib.util
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('left_live_launcher',ROOT/'start_left_live_compare.py')
launcher=importlib.util.module_from_spec(spec);spec.loader.exec_module(launcher)


class LauncherTest(unittest.TestCase):
    def test_port_mapping_requires_unchanged_installation_and_calibration(self):
        from orin_rectified_stream.start_pro_rectified import capture_ports,sender_matches_ports
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'configs').mkdir()
            self.assertEqual(capture_ports(root),[8,0])
            raw=root/'configs/orin_pro_raw_fisheye_stereo_20261019.json';raw.write_text('{}')
            value=dict(schema='orin_pro_camera_ports_v1',capture_sensors=[1,0],installation_unchanged_confirmed=True,
                       source_calibration_sha256=hashlib.sha256(raw.read_bytes()).hexdigest())
            path=root/'configs/orin_pro_camera_ports.json';path.write_text(json.dumps(value))
            self.assertEqual(capture_ports(root),[1,0])
            for changes in [dict(capture_sensors=[1,1]),dict(installation_unchanged_confirmed=False),dict(source_calibration_sha256='wrong')]:
                path.write_text(json.dumps(dict(value,**changes)))
                with self.assertRaises(ValueError):capture_ports(root)
        self.assertFalse(sender_matches_ports('123 python pro_sender.py',[1,0]))
        self.assertTrue(sender_matches_ports('123 python pro_sender.py',[8,0]))
        self.assertTrue(sender_matches_ports('123 python pro_sender.py --sensors 1 0',[1,0]))
        self.assertFalse(sender_matches_ports('123 python pro_sender.py --sensors 1 6',[1,0]))

    def test_metadata_age_and_bad_files(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'status.json'
            self.assertFalse(launcher.metadata_fresh(path,3000000000))
            path.write_text(json.dumps(dict(local_receive_monotonic_ns=2000000000)))
            self.assertTrue(launcher.metadata_fresh(path,3000000000))
            self.assertFalse(launcher.metadata_fresh(path,5000000000))
            self.assertFalse(launcher.metadata_fresh(path,1000000000))
            path.write_text('bad');self.assertFalse(launcher.metadata_fresh(path))

    def test_old_terminal_commands_are_not_live_python_modules(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            commands=[['/usr/bin/python3','/usr/bin/gnome-terminal','--','python3','-m','interactive_ball_calibration.live_left_compare'],
                      ['/usr/bin/python3','/usr/bin/gnome-terminal','--wait','--','bash','-c',
                       '/usr/bin/python3 -u -m interactive_ball_calibration.live_left_compare'],
                      ['/bin/bash','-c','python3 -m interactive_ball_calibration.live_left_compare'],
                      ['/usr/bin/gnome-terminal.real','--','python3','-m','interactive_ball_calibration.live_left_compare']]
            for index,argv in enumerate(commands):
                process=root/str(100+index);process.mkdir()
                (process/'cmdline').write_bytes(b'\0'.join(s.encode() for s in argv)+b'\0')
            self.assertFalse(launcher.comparison_running(root))
            process=root/'200';process.mkdir()
            (process/'cmdline').write_bytes(b'/usr/bin/python3\0-u\0-m\0interactive_ball_calibration.live_left_compare\0')
            self.assertTrue(launcher.comparison_running(root))
            (process/'cmdline').write_bytes(b'')
            self.assertFalse(launcher.comparison_running(root))

    def test_existing_view_is_not_restarted(self):
        with tempfile.TemporaryDirectory() as directory,patch.object(launcher,'ROOT',Path(directory)),\
             patch.object(launcher,'comparison_running',return_value=True),\
             patch.object(launcher,'run_live') as live,patch.object(launcher,'prepare_sender') as camera:
            launcher.main();live.assert_not_called();camera.assert_not_called()

    def test_ready_camera_is_reused(self):
        with tempfile.TemporaryDirectory() as directory,patch.object(launcher,'ROOT',Path(directory)),\
             patch.object(launcher,'comparison_running',return_value=False),\
             patch.object(launcher,'metadata_fresh',return_value=True),\
             patch.object(launcher,'run_live') as live,patch.object(launcher,'prepare_sender') as camera:
            launcher.main();live.assert_called_once();camera.assert_not_called()

    def test_launcher_lock_is_released_when_view_returns(self):
        with tempfile.TemporaryDirectory() as directory,patch.object(launcher,'ROOT',Path(directory)),             patch.object(launcher,'comparison_running',return_value=False),             patch.object(launcher,'metadata_fresh',return_value=True),             patch.object(launcher,'run_live') as live:
            launcher.main();launcher.main()
            self.assertEqual(live.call_count,2)

    def test_running_sender_reuse_does_not_upload_or_stop_camera(self):
        from orin_rectified_stream import start_pro_rectified as camera
        client=Mock()
        hashes='\n'.join(hashlib.sha256((camera.ROOT/name).read_bytes()).hexdigest()+'  file' for name in
            ('configs/orin_pro_raw_fisheye_stereo_20261019.json','configs/orin_pro_rectified_stereo_20261019.json'))
        with patch.dict(camera.os.environ,{'ORIN_PRO_PASSWORD':'test-only'}),             patch.object(camera.paramiko,'SSHClient',return_value=client),             patch.object(camera,'remote',side_effect=['123 python pro_sender.py --sensors 1 0\n',hashes]) as remote:
            camera.prepare_sender(reuse_running=True)
        self.assertEqual(remote.call_count,2)
        client.open_sftp.assert_not_called();client.close.assert_called_once()

    def test_cold_start_prepares_camera_and_cleans_bootstrap_viewer(self):
        preview=Mock();preview.poll.return_value=0
        with tempfile.TemporaryDirectory() as directory,patch.object(launcher,'ROOT',Path(directory)),\
             patch.object(launcher,'comparison_running',return_value=False),\
             patch.object(launcher.subprocess,'Popen',return_value=preview),\
             patch.object(launcher,'metadata_fresh',side_effect=[False,True]),\
             patch.object(launcher,'run_live') as live,patch.object(launcher,'prepare_sender') as camera:
            launcher.main();camera.assert_called_once();live.assert_called_once();preview.wait.assert_called_once()


if __name__=='__main__':unittest.main()
