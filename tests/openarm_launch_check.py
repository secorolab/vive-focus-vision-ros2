#!/usr/bin/env python3
"""Calibration launch must stop its own servers on success, failure and cancellation."""
from pathlib import Path
import json
import tempfile
import yaml
import signal
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import openarm_sim


class CalibrationLaunch(unittest.TestCase):
    def exercise(self, error=None, slow_shutdown=False):
        process = MagicMock(pid=12345)
        process.poll.return_value = None
        if slow_shutdown:
            process.wait.side_effect = [subprocess.TimeoutExpired('launch', 15), 0]
        args = SimpleNamespace(delay=20, arm='left')
        with patch.object(openarm_sim.subprocess, 'Popen', return_value=process) as start, \
             patch.object(openarm_sim, 'align', side_effect=error) as capture, \
             patch.object(openarm_sim.os, 'killpg') as stop:
            if error is None:
                openarm_sim.calibrate_before_launch(['ros2', 'launch', 'test'], args)
            else:
                with self.assertRaises(type(error)):
                    openarm_sim.calibrate_before_launch(['ros2', 'launch', 'test'], args)
            start.assert_called_once_with(['ros2', 'launch', 'test', 'calibration_only:=true'],
                                          start_new_session=True)
            capture.assert_called_once_with(args)
            self.assertEqual(stop.call_args_list[0].args, (12345, signal.SIGINT))
            if slow_shutdown:
                self.assertEqual(stop.call_args_list[1].args, (12345, signal.SIGKILL))
                self.assertEqual(process.wait.call_count, 2)
            else:
                process.wait.assert_called_once_with(timeout=15)

    def test_configure_keeps_independent_calibrations_and_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            model = folder / 'openarm_v1_controlled.xml'
            model.write_text('existing model')
            right = {'tool_from_controller_rpy': [10, 20, 30], 'quaternion_xyzw': [0, 0, 0, 1]}
            left = {'tool_from_controller_rpy': [-10, -20, -30], 'quaternion_xyzw': [1, 0, 0, 0]}
            (folder / 'controller_alignment.json').write_text(json.dumps(right))
            (folder / 'controller_alignment_left.json').write_text(json.dumps(left))
            openarm_sim.configure(SimpleNamespace(output=folder))
            saved = yaml.safe_load((folder / 'teleop.yaml').read_text())
            arms = saved['vive_teleop']['ros__parameters']['teleop']
            self.assertEqual(arms['right']['tool_from_controller_rpy'], right['tool_from_controller_rpy'])
            self.assertEqual(arms['left']['tool_from_controller_rpy'], left['tool_from_controller_rpy'])
            params = saved['vive_scene']['ros__parameters']
            self.assertEqual(params['openarm.controller_to_tool_xyzw'], right['quaternion_xyzw'])
            self.assertEqual(params['openarm.left.controller_to_tool_xyzw'], left['quaternion_xyzw'])
            self.assertEqual(model.read_text(), 'existing model')

    def test_success_stops_calibration_servers_before_restart(self):
        self.exercise()

    def test_failure_propagates_after_cleanup(self):
        self.exercise(subprocess.CalledProcessError(1, 'align'))

    def test_cancel_cleans_up(self):
        self.exercise(KeyboardInterrupt())

    def test_stuck_launch_is_reaped(self):
        self.exercise(slow_shutdown=True)


if __name__ == '__main__':
    unittest.main()
