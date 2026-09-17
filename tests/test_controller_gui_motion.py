"""Exercise motion intent and cancellation without moving real hardware."""
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import queue

sys.path.insert(0, str(Path(__file__).parents[1] / 'software' / 'STM32ControllerGUI'))
from stm32_controller_gui import ControllerGui, SerialWorker, CommandRequest, parse_pose


def test_jogs_accumulate_at_destination_not_old_measured_pose():
    app = SimpleNamespace(pose=(0, 0, 0), target=(1, 2, 3),
                          jog_var=SimpleNamespace(get=lambda: '.1'),
                          feed_var=SimpleNamespace(get=lambda: '1'))
    app.schedule_target = lambda value: setattr(app, 'target', value)
    ControllerGui.jog(app, 0, 1)
    ControllerGui.jog(app, 0, 1)
    assert abs(app.target[0] - 1.2) < 1e-9
    assert app.target[1:] == (2, 3)


def test_cancel_queue_does_not_leave_old_moves_after_stop():
    events = queue.Queue()
    worker = SerialWorker(events)
    worker.pending = 3  # one active request, two waiting
    worker.active_command = 'M57'
    worker.requests.put(CommandRequest('G0 X1', 5, False))
    worker.requests.put(CommandRequest('G0 X2', 5, False))
    worker.cancel_queued()
    assert worker.pending == 1
    assert worker.requests.empty()
    assert events.get()[2] == 'cancelled'
    assert events.get()[2] == 'cancelled'


def test_pose_reply_uses_measurement_before_destination():
    assert parse_pose(['X0.1 Y0.2 Z0.3', 'Destination: X1 Y2 Z3']) == (.1, .2, .3)


def test_stop_cancels_debounce_and_queue_before_hold():
    calls = []
    app = SimpleNamespace(calibration_active=False, target=None,
        worker=SimpleNamespace(active_command='', connected=True,
            cancel_queued=lambda: calls.append('queue')),
        cancel_target=lambda: calls.append('target'),
        send=lambda command: calls.append(command), destination_var=Mock())
    ControllerGui.stop_hold(app)
    assert calls == ['target', 'queue', 'M0']


def test_rapid_targets_replace_pending_timer():
    app = SimpleNamespace(calibration_active=False, driver_active=True,
        worker=SimpleNamespace(connected=True), target=None, target_revision=0,
        target_timer=None, destination_var=Mock(), flush_target=Mock(),
        after=Mock(side_effect=[101, 102]), after_cancel=Mock())
    ControllerGui.schedule_target(app, (1, 0, 0))
    ControllerGui.schedule_target(app, (2, 0, 0))
    assert app.target == (2, 0, 0)
    assert app.target_timer == 102
    app.after_cancel.assert_called_once_with(101)


def test_old_rejection_cannot_cancel_newer_destination():
    app = SimpleNamespace(target=(1, 0, 0), target_revision=1,
        target_inflight=False, worker=SimpleNamespace(connected=True),
        feed_var=SimpleNamespace(get=lambda: '1'), send=Mock(),
        cancel_target=Mock(), destination_var=Mock())
    ControllerGui.flush_target(app)
    callback = app.send.call_args.kwargs['callback']
    app.target = (2, 0, 0)
    app.target_revision = 2
    callback('error', ['rejected'])
    app.cancel_target.assert_not_called()
    assert app.target == (2, 0, 0)
    assert not app.target_inflight
