import argparse
import configparser
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from interactive_ball_calibration import pro_capture as module


class LeftCaptureTest(unittest.TestCase):
    def make_capture(self,directory):
        root=Path(directory);(root/'data').mkdir();(root/'runtime').mkdir()
        args=argparse.Namespace(arm='left',robot_ip='192.168.12.190',mdh=str(module.ROOT/'configs/1004.ini'),
                                stereo=str(module.ROOT/'configs/orin_pro_rectified_stereo_20261019.json'))
        with patch.object(module,'ROOT',root),patch.object(module,'digest',return_value='hash'):
            capture=module.Capture(args)
        return capture

    def test_left_config_and_provenance(self):
        with tempfile.TemporaryDirectory() as directory:
            c=self.make_capture(directory)
            self.assertEqual(c.arm_index,0);self.assertEqual(c.prefix,'R.A0')
            self.assertTrue(c.directory.name.startswith('left_drag_replay_'))
            self.assertEqual(c.provenance['arm_side'],'left')
            self.assertTrue(c.provenance['mdh_source'].endswith(':R.A0'))
            np.testing.assert_array_equal(c.rows,module.mdh_rows(c.args.mdh,0))
            config=configparser.ConfigParser();config.read(c.args.mdh)
            config.set('R.A0.L0.DH','A','123')
            fixture=Path(directory)/'different_sides.ini'
            with fixture.open('w') as f:config.write(f)
            self.assertEqual(module.mdh_rows(fixture,0)[0,0],123)
            self.assertNotEqual(module.mdh_rows(fixture,1)[0,0],123)

    def test_left_connect_feedback_and_idle_never_target_right(self):
        with tempfile.TemporaryDirectory() as directory:
            c=self.make_capture(directory);robot=Mock();robot._connected=True
            robot.get_sdk_version.return_value=robot.get_controller_version.return_value=module.VERSION
            robot.param_get_int.side_effect=lambda name:(0,7) if name=='R.A0.BASIC.Dof' else self.fail(name)
            config=configparser.ConfigParser();config.read(c.args.mdh)
            def parameter(name):
                self.assertTrue(name.startswith('R.A0.'));section,key=name.rsplit('.',1)
                return 0,config.getfloat(section,key)
            robot.param_get_float.side_effect=parameter
            serial=[0]
            def feedback():
                serial[0]+=1
                return dict(frame_serial=serial[0],arms=[dict(fb=dict(fb_pos=[0.]*7,fb_vel=[0.]*7)),
                                                       dict(fb=dict(fb_pos=[99.]*7,fb_vel=[99.]*7))])
            robot.get_rt_dict.side_effect=feedback;robot.check_link_state.return_value=1
            robot.current_state.return_value=0;robot.switch_to_idle.return_value=0
            sdk=SimpleNamespace(GentoRobot=Mock(return_value=robot),state_map={0:'IDLE'},
                FXObjType=SimpleNamespace(OBJ_ARM0=0,OBJ_ARM1=1),FXLogMask=SimpleNamespace(FX_LOG_INFO_FLAG=1))
            with patch.object(module.importlib,'import_module',return_value=sdk),patch.object(module.time,'sleep'):
                c.connect()
            np.testing.assert_array_equal(c.feedback()[0],np.zeros(7))
            self.assertEqual(c.arm,0);self.assertTrue(all(call.args==(0,) for call in robot.current_state.call_args_list))
            c.owned=True;c.idle();robot.switch_to_idle.assert_called_once_with(0,2000)

    def test_right_trajectory_is_rejected_for_left(self):
        with tempfile.TemporaryDirectory() as directory:
            c=self.make_capture(directory);manifest=dict(c.provenance,schema='head_eye_pro_taught_path_v1',
                complete=True,arm_side='right',arm_index=1,waypoints=[])
            path=Path(directory)/'right.json';path.write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError,'provenance mismatch'):
                module.load_replay_path(path,c.provenance,np.array([[-170,170]]*7))

    def test_left_replay_commands_and_samples_left_only(self):
        with tempfile.TemporaryDirectory() as directory:
            c=self.make_capture(directory);c.arm=0;c.limits=np.array([[-170,170]]*7)
            c.robot=Mock();c.robot.switch_to_position_mode.return_value=0
            c.robot.runtime_set_joint_pos_cmd.return_value=0;c.robot.switch_to_idle.return_value=0
            c.robot._connected=True;c.state=lambda:'IDLE';position=[51.]+[0.]*6
            c.feedback=lambda:(np.array(position),np.zeros(7),1)
            def command(arm,target):self.assertEqual(arm,0);position[:]=target;return 0
            c.robot.runtime_set_joint_pos_cmd.side_effect=command;c.wait_arm=Mock();c.status=Mock()
            c.sample=lambda target:([],0)
            with patch('builtins.print'):
                c.replay([dict(q7_deg=[3.*i]+[0.]*6) for i in range(18)])
            c.robot.switch_to_position_mode.assert_called_once_with(0,2000,5,5)
            c.robot.switch_to_idle.assert_called_once_with(0,2000)
            rows=[json.loads(line) for line in c.output.read_text().splitlines()]
            self.assertEqual(len(rows),18);self.assertTrue(all(r['arm_index']==0 and r['arm_side']=='left' for r in rows))


if __name__=='__main__':unittest.main()
