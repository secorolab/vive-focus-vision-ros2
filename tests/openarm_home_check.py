"""Supervisor readiness checks; no ROS imports, launch or hardware access."""
import importlib.util
from pathlib import Path
from unittest.mock import Mock, patch
spec=importlib.util.spec_from_file_location('home',Path(__file__).parents[1]/'scripts/openarm_home.py')
home=importlib.util.module_from_spec(spec); spec.loader.exec_module(home)
health={'right':(True,10.0),'left':(True,10.0)}
assert home.feedback_ready(10.05,10.0,health)
assert not home.feedback_ready(10.2,10.0,health)
assert not home.feedback_ready(10.05,9.0,health)
assert not home.feedback_ready(10.05,10.0,{'right':(True,10.0)})
assert not home.feedback_ready(10.05,10.0,{**health,'left':(False,10.0)})
assert not home.feedback_ready(10.05,10.0,{**health,'left':(True,9.0)})
assert not home.feedback_ready(10.05,11.0,health)
process=Mock(pid=123); process.poll.return_value=None
with patch.object(home.os,'killpg') as send:
    home.stop_owned(process)
    send.assert_called_once_with(123,home.signal.SIGINT)
    process.wait.assert_called_once_with(timeout=12)
process.poll.return_value=0
with patch.object(home.os,'killpg') as send:
    home.stop_owned(process)
    send.assert_not_called()
print('Local home readiness and owned-process cleanup checks passed.')
