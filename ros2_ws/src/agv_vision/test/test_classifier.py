"""표지판 분류 — 확정 규칙, 전처리, 가짜 인터프리터로 predict (모델 파일 없이)."""
import numpy as np
import pytest

from agv_vision.classifier import (BACKGROUND, DEFAULT_LABEL_MAP, TfliteClassifier, decide, load_labels, map_label,
                                   parse_label_map, preprocess)


def test_decide_needs_three_confident_same_labels():
    r = [("sign_work", 0.9), ("sign_work", 0.95), ("sign_park", 0.9), ("sign_work", 0.7), ("sign_work", 0.85)]
    label, score = decide(r)
    assert label == "sign_work" and score == pytest.approx((0.9 + 0.95 + 0.85) / 3)


@pytest.mark.parametrize("results", [
    [("sign_work", 0.9)] * 2 + [("sign_park", 0.9)] * 2,       # 같은 라벨이 3개 미만
    [("sign_work", 0.7)] * 5,                                   # 신뢰도 미달
    [(BACKGROUND, 0.99)] * 5,                                   # 배경은 확정하지 않음
    [],
])
def test_decide_rejects(results):
    assert decide(results) is None


def test_preprocess_center_crop_rgb_float():
    bgr = np.zeros((240, 320, 3), np.uint8)
    bgr[:, :40] = (255, 0, 0)                                   # 왼쪽 띠 = 파랑(BGR) — 가운데 크롭으로 잘림
    bgr[100:140, 140:180] = (0, 0, 255)                         # 가운데 빨강
    x = preprocess(bgr, 224, np.float32)
    assert x.shape == (1, 224, 224, 3) and x.dtype == np.float32
    assert x[0, 112, 112].tolist() == pytest.approx([1.0, -1.0, -1.0])   # RGB 빨강, -1~1
    assert x[0, 112, 0].tolist() == pytest.approx([-1.0, -1.0, -1.0])
    assert preprocess(bgr, 224, np.uint8).dtype == np.uint8


def test_load_labels(tmp_path):
    f = tmp_path / "labels.txt"
    f.write_text("0 sign_work\n1 sign_drop_a\n\n2 background\n", encoding="utf-8")
    assert load_labels(str(f)) == ["sign_work", "sign_drop_a", "background"]


class FakeInterpreter:
    def __init__(self, scores, out_dtype=np.float32, quant=(0.0, 0)):
        self.scores, self.got = np.array([scores], dtype=out_dtype), None
        self.out_dtype, self.quant = out_dtype, quant

    def get_input_details(self):
        return [{"index": 0, "shape": [1, 224, 224, 3], "dtype": np.float32}]

    def get_output_details(self):
        return [{"index": 1, "dtype": self.out_dtype, "quantization": self.quant}]

    def set_tensor(self, i, x):
        self.got = x

    def invoke(self):
        pass

    def get_tensor(self, i):
        return self.scores


def test_predict_picks_top_label(tmp_path):
    f = tmp_path / "labels.txt"
    f.write_text("0 sign_work\n1 sign_park\n2 background\n", encoding="utf-8")
    c = TfliteClassifier("x", str(f), interpreter=FakeInterpreter([0.1, 0.85, 0.05]))
    label, score = c.predict(np.zeros((480, 640, 3), np.uint8))
    assert label == "sign_park" and score == pytest.approx(0.85) and c.it.got.shape == (1, 224, 224, 3)


def test_predict_dequantizes_uint8_output(tmp_path):
    f = tmp_path / "labels.txt"
    f.write_text("0 sign_work\n1 background\n", encoding="utf-8")
    c = TfliteClassifier("x", str(f), interpreter=FakeInterpreter([230, 25], np.uint8, (1 / 255, 0)))
    label, score = c.predict(np.zeros((240, 320, 3), np.uint8))
    assert label == "sign_work" and score == pytest.approx(230 / 255)


def test_label_map_default_matches_trained_model():
    """2026-11-06 Teachable Machine 모델 labels.txt (HOME BOTTLE BOX LOAD none) → 표지판 라벨."""
    m = parse_label_map(DEFAULT_LABEL_MAP)
    assert [map_label(x, m) for x in ("LOAD", "BOTTLE", "BOX", "HOME", "none")] == \
        ["sign_work", "sign_drop_a", "sign_drop_b", "sign_park", BACKGROUND]


def test_label_map_ignores_case_and_keeps_unknown():
    m = parse_label_map(["Home:sign_park", "bad", ":x", "y:"])
    assert m == {"home": "sign_park"}
    assert map_label("HOME", m) == "sign_park" and map_label("CAT", m) == "CAT"


def test_mapped_none_is_background_not_confirmed():
    m = parse_label_map(DEFAULT_LABEL_MAP)
    results = [(map_label("none", m), 0.99)] * 5
    assert decide(results) is None


def test_repo_model_labels_all_mapped():
    """저장소 models/labels.txt의 모든 클래스가 표지판 라벨이나 background로 바뀜."""
    import os
    from agv_protocol.types import Sign
    path = os.path.join(os.path.dirname(__file__), "..", "..", "agv_bringup", "models", "labels.txt")
    m = parse_label_map(DEFAULT_LABEL_MAP)
    mapped = {map_label(x, m) for x in load_labels(path)}
    assert mapped == {s.value for s in Sign} | {BACKGROUND}
