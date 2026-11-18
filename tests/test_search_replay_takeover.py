"""Replay truthfulness and recovery regressions from the second review."""
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import pytest

from app.search_replay import storage
from app.search_replay.manager import CaptureInfo, ReplayManager
from app.search_replay.models import PoseRow
from app.search_replay.recording import Recording, load_recording
from app.search_replay.renderer import MAP_FREE, ReplayRenderer
from tests.search_replay_fixture import make_recording
from tests.test_search_replay_review_regressions import _config, _FakeHandle

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/go2w"))
import record_search_replay_ros as collector


def transform(stamp):
    sec = int(stamp)
    return NS(header=NS(frame_id="map", stamp=NS(sec=sec, nanosec=int((stamp-sec)*1e9))),
              transform=NS(translation=NS(x=0., y=0., z=0.),
                           rotation=NS(x=0., y=0., z=0., w=1.)))


def map_message(value):
    return NS(data=[value]*4, header=NS(frame_id="map", stamp=NS(sec=1, nanosec=0)),
              info=NS(width=2, height=2, resolution=.1,
                      origin=NS(position=NS(x=0., y=0., z=0.),
                                orientation=NS(x=0., y=0., z=0., w=1.))))


@pytest.mark.parametrize("values", [(0, 100, 0), (0, 100, 0, 100, 0), (0, 100, 50)])
def test_rate_limited_map_finalization_keeps_latest_source(tmp_path, values):
    sid = "search_takeover_maps"
    rec = collector.Recorder(tmp_path / sid, _config(tmp_path), "", 1)
    rec.configure_session(sid, "rec_test")
    for i, value in enumerate(values):
        with patch.object(collector, "_now", return_value=(100+i*.1, 100+i*.1)):
            rec.on_map(map_message(value))
    rec.store_pending_map(102); rec.flush()
    recording = load_recording(tmp_path / sid)
    grid = recording.map_grid(recording.last_map())
    assert np.all(grid == values[-1])
    assert recording.missing_maps == 0


@pytest.mark.parametrize("topic_valid", [True, False])
def test_tf_lookup_recovers_without_rewriting_topic_verdict(tmp_path, topic_valid):
    rec = collector.Recorder(tmp_path / "search_tf", _config(tmp_path), "", 1)
    rec.configure_session("search_tf", "rec_test")
    rec.add_localization(9.9, 999, {"valid": topic_valid}, "topic")
    class Lookup(Exception):
        pass
    class Buffer:
        failed = True
        def lookup_transform(self, *args, **kwargs):
            if self.failed:
                raise Lookup("temporary gap")
            return transform(100.1)
    buf = Buffer()
    args = (rec, buf, lambda: None, lambda **kwargs: None, Lookup, Lookup, Lookup)
    collector._sample_pose(*args, stamp=10., wall=1000.)
    buf.failed = False
    collector._sample_pose(*args, stamp=10.1, wall=1000.1)
    rows = [row for row in rec._pose_buffer if row.get("kind") == "pose"]
    assert rec._topic_valid is topic_valid
    assert rec._tf_lookup_valid is True
    assert rows[-1]["valid"] is topic_valid


def test_prolonged_tf_gap_has_bounded_notes_and_preserves_failure_evidence(tmp_path):
    rec=collector.Recorder(tmp_path/'search_long_gap',_config(tmp_path),'',1)
    rec.configure_session('search_long_gap','rec_test')
    class Lookup(Exception):pass
    class Buffer:
        def lookup_transform(self,*args,**kwargs):raise Lookup('missing')
    args=(rec,Buffer(),lambda:None,lambda **kwargs:None,Lookup,Lookup,Lookup)
    for i in range(3000):
        collector._sample_pose(*args,stamp=10.+i*.1,wall=1000.+i*.1)
    assert rec.counts['tf_failures']==3000
    assert rec._tf_gap_start==10.
    assert rec._tf_lookup_valid is False
    assert len([n for n in rec.notes if n.startswith('TF 中断')])==1
    assert sum(len(n) for n in rec.notes)<500
    # Recovery retains the full elapsed duration rather than truncating it.
    buf=NS(lookup_transform=lambda *a,**kw:transform(1300.))
    collector._sample_pose(rec,buf,*args[2:],stamp=310.,wall=1300.)
    assert rec._tf_gap_start is None and rec._tf_lookup_valid
    assert any('300.0s' in n for n in rec.notes)


@pytest.mark.parametrize("explicit_span", [True, False])
def test_known_invalid_tail_never_claims_current_location(tmp_path, explicit_span):
    poses = [PoseRow(10+i*.1, 1000+i*.1, i*.02, 0, 0,
                     valid=explicit_span or i < 5) for i in range(10)]
    recording = Recording(tmp_path, tmp_path, {}, poses=poses,
                          invalid_spans=[(10.5, 10.9)] if explicit_span else [])
    renderer = ReplayRenderer(recording, _config(tmp_path)); renderer.prepare()
    assert renderer.pose_at(10.4) is not None
    assert renderer.pose_at(10.8) is None
    texts = []
    renderer._text = lambda *args, **kwargs: texts.append(str(args[2]))
    renderer._draw_footer(renderer._base_canvas(), 10.8)
    assert not any("位姿：有效" in text for text in texts)


def test_zero_valid_trajectory_is_playable_partial(tmp_path):
    sid = "search_takeover_invalid"
    session = make_recording(tmp_path, session_id=sid, seconds=1., camera_fps=1.)
    path = session / "replay/poses.jsonl"
    rows, _ = storage.iter_jsonl(path)
    for row in rows:
        if row.get("kind") == "pose":
            row["valid"] = False
    path.write_text("".join(json.dumps(row)+"\n" for row in rows))
    manager = ReplayManager(_config(tmp_path), sessions_root=tmp_path)
    result = manager.render_offline(sid)
    status = manager.status(sid)
    assert result["ok"] and status["playable"]
    assert status["status"] == "PARTIAL"
    assert status["quality"] == "PARTIAL"
    assert any("有效定位轨迹" in reason for reason in status["reasons"])


def test_finish_retries_once_capacity_is_available(tmp_path):
    sid = "search_takeover_queue"
    session = make_recording(tmp_path, session_id=sid, seconds=1., camera_fps=1.)
    manager = ReplayManager(_config(tmp_path, render_queue_capacity=4), sessions_root=tmp_path)
    manager.start = lambda: None
    info = CaptureInfo(sid, session, "rec_test", time.monotonic())
    handle = _FakeHandle(info); handle.start(); info.handle = handle
    manager._collectors[sid] = info
    for i in range(manager._finalize.maxsize):
        manager._finalize.put_nowait(("occupied", str(i)))
    manager.finish_session(sid, {"status": "FAILED", "finish_reason": "worker_exit"})
    while not manager._finalize.empty():
        manager._finalize.get_nowait()
    manager._recover_session(sid, session)
    item = manager._finalize.get_nowait()
    assert item[0:2] == ("finish", sid)
    assert item[2]["finish_reason"] == "worker_exit"
    manager._handle_finish(sid, item[2])
    assert handle.stopped.is_set() and sid not in manager._collectors
    manager.finish_session(sid, {"status": "FAILED"})
    assert manager._finalize.empty(), "duplicate terminal hook must not finish twice"


def test_finalization_write_failure_is_retried_by_running_worker(tmp_path):
    import threading
    manager=ReplayManager(_config(tmp_path),sessions_root=tmp_path)
    completed=threading.Event();attempts=[]
    def finish(sid,snapshot):
        attempts.append((sid,snapshot['finish_reason']))
        if len(attempts)==1:
            raise OSError('temporary disk write failure')
        completed.set()
    manager._handle_finish=finish
    manager.start()
    try:
        manager.finish_session('search_retry_write',{'status':'FINISHED','finish_reason':'test_terminal'})
        assert completed.wait(3),'terminal hook was lost after a finalizer exception'
        assert attempts==[('search_retry_write','test_terminal')]*2
    finally:
        manager.shutdown(timeout=2)


def test_corrupt_last_grid_falls_back_in_video_and_final_png(tmp_path):
    from PIL import Image
    sid = "search_takeover_corrupt"
    session = make_recording(tmp_path, session_id=sid, seconds=2., camera_fps=1.)
    recording = load_recording(session)
    recording.last_map().path.write_bytes(b"BROKEN_NPZ")
    manager = ReplayManager(_config(tmp_path), sessions_root=tmp_path)
    result = manager.render_offline(sid)
    assert result["ok"] and result["status"] == "PARTIAL"
    assert any("无法解码" in reason for reason in result["reasons"])
    with Image.open(session / "replay/artifacts/final_map.png") as image:
        rgb = np.array(image.convert("RGB"))
    assert np.all(rgb == MAP_FREE, axis=-1).sum() > 1000


def test_archiving_previous_video_keeps_published_file(tmp_path):
    manager = ReplayManager(_config(tmp_path), sessions_root=tmp_path)
    output = tmp_path / "artifacts/replay.mp4"
    output.parent.mkdir(); output.write_bytes(b"previous playable artifact")
    archived = manager._archive_previous(output, output.parent, "old", [])
    assert archived and output.read_bytes() == b"previous playable artifact"
    assert (output.parent / archived["path"]).read_bytes() == output.read_bytes()
