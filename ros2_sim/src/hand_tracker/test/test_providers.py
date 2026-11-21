import sys
import types

import numpy as np
import pytest

from hand_tracker.trackers.providers import CPU, CUDA, DML, provider_options, select_providers


def test_auto_prefers_cuda_then_dml_then_cpu():
    assert select_providers([CPU, DML, CUDA]) == [CUDA, DML, CPU]
    assert select_providers([DML, CPU]) == [DML, CPU]
    assert select_providers([CPU]) == [CPU]


def test_explicit_device_and_missing_provider_falls_to_cpu():
    assert select_providers([CPU, DML], "dml") == [DML, CPU]
    assert select_providers([CPU, DML], "cpu") == [CPU]
    assert select_providers([CPU], "dml") == [CPU]          # not offered by this build -> CPU
    with pytest.raises(ValueError):
        select_providers([CPU], "tpu")


def test_dml_gets_device_id():
    assert provider_options([DML, CPU]) == [{"device_id": 0}, {}]


class FakeSession:
    fail_on = None

    def __init__(self, path, sess_options=None, providers=None, provider_options=None):
        self.providers = providers
        if providers[0] == self.fail_on:
            raise RuntimeError("driver exploded")

    def get_inputs(self):
        return [types.SimpleNamespace(name="images", shape=[1, 3, 64, 64])]

    def get_providers(self):
        return self.providers

    def run(self, *a):
        return [np.zeros((1, 68, 4))]


def fake_ort(available, fail_on=None):
    FakeSession.fail_on = fail_on
    return types.SimpleNamespace(
        get_available_providers=lambda: available, InferenceSession=FakeSession,
        SessionOptions=lambda: types.SimpleNamespace(), ExecutionMode=types.SimpleNamespace(ORT_SEQUENTIAL=1))


def make(monkeypatch, tmp_path, available, fail_on=None, **kw):
    from hand_tracker.trackers.yolo_tracker import YoloTracker
    monkeypatch.setitem(sys.modules, "onnxruntime", fake_ort(available, fail_on))
    m = tmp_path / "m.onnx"; m.write_bytes(b"x")
    return YoloTracker(model=str(m), **kw)


def test_tracker_uses_dml_when_available(monkeypatch, tmp_path):
    assert make(monkeypatch, tmp_path, [DML, CPU]).active_provider == DML


def test_tracker_falls_back_to_cpu_when_dml_fails(monkeypatch, tmp_path, capsys):
    t = make(monkeypatch, tmp_path, [DML, CPU], fail_on=DML)
    assert t.active_provider == CPU
    assert "falling back to CPU" in capsys.readouterr().err


def test_tracker_cpu_only_build(monkeypatch, tmp_path):
    assert make(monkeypatch, tmp_path, [CPU], device="dml").active_provider == CPU
