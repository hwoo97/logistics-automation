"""표지판 분류 (② 순수 로직) — Teachable Machine TFLite 모델을 LiteRT로 추론, 여러 프레임으로 확정 (functional_design F7).

전처리는 Teachable Machine과 같게: 가운데 정사각 크롭 → 224×224 → BGR → RGB → (부동소수 모델) 픽셀/127.5 − 1.
확정: 프레임마다 (라벨, 신뢰도) → 신뢰도 min_score 이상인 같은 라벨이 need번 이상, background가 아니면 확정 (tm_classes.md 5절).
"""
from collections import Counter

import cv2
import numpy as np

BACKGROUND = "background"
# 모델 클래스 이름 → 표지판 라벨 (2026-11-06 학습 모델: labels.txt = HOME BOTTLE BOX LOAD none)
# LOAD = 상차 지점, BOTTLE = 원(병) 하차 = DROP_A, BOX = 사각(상자) 하차 = DROP_B, HOME = 대기장소, none = 표지판 없음
DEFAULT_LABEL_MAP = ("LOAD:sign_work", "BOTTLE:sign_drop_a", "BOX:sign_drop_b", "HOME:sign_park", "none:background")


def parse_label_map(items) -> dict[str, str]:
    """"모델이름:표지판라벨" 목록 → {모델이름(소문자): 표지판라벨}. 형식이 틀린 항목은 건너뜀."""
    out = {}
    for item in items or ():
        name, sep, sign = str(item).partition(":")
        if sep and name.strip() and sign.strip():
            out[name.strip().lower()] = sign.strip()
    return out


def map_label(label: str, mapping: dict[str, str]) -> str:
    """모델 클래스 이름을 표지판 라벨로 (대소문자 무시). 표에 없으면 그대로."""
    return mapping.get(label.lower(), label)


def load_labels(path: str) -> list[str]:
    """Teachable Machine labels.txt — 줄마다 "0 sign_work"."""
    labels = []
    with open(path, encoding="utf-8") as fp:
        for line in fp:
            parts = line.strip().split(maxsplit=1)
            if parts:
                labels.append(parts[-1])
    return labels


def preprocess(bgr: np.ndarray, size: int, dtype) -> np.ndarray:
    h, w = bgr.shape[:2]
    s = min(h, w)
    y0, x0 = (h - s) // 2, (w - s) // 2
    img = cv2.resize(bgr[y0:y0 + s, x0:x0 + s], (size, size), interpolation=cv2.INTER_AREA)
    x = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    x = x.astype(np.float32) / 127.5 - 1.0 if dtype == np.float32 else x.astype(dtype)
    return x[None, ...]


def decide(results: list[tuple[str, float]], min_score: float = 0.8, need: int = 3) -> tuple[str, float] | None:
    """프레임별 (라벨, 신뢰도) → 확정 (라벨, 평균 신뢰도) 또는 None."""
    good = [(label, score) for label, score in results if score >= min_score]
    if not good:
        return None
    label, count = Counter(label for label, _ in good).most_common(1)[0]
    if count < need or label == BACKGROUND:
        return None
    return label, float(np.mean([score for lb, score in good if lb == label]))


def _interpreter(path: str):
    try:
        from ai_edge_litert.interpreter import Interpreter
    except ImportError:
        try:
            from tflite_runtime.interpreter import Interpreter
        except ImportError:
            from tensorflow.lite import Interpreter
    it = Interpreter(model_path=path)
    it.allocate_tensors()
    return it


class TfliteClassifier:
    def __init__(self, model_path: str, labels_path: str, interpreter=None):
        self.it = interpreter or _interpreter(model_path)
        self.inp = self.it.get_input_details()[0]
        self.out = self.it.get_output_details()[0]
        self.size = int(self.inp["shape"][1])
        self.labels = load_labels(labels_path)

    def predict(self, bgr: np.ndarray) -> tuple[str, float]:
        self.it.set_tensor(self.inp["index"], preprocess(bgr, self.size, self.inp["dtype"]))
        self.it.invoke()
        y = self.it.get_tensor(self.out["index"])[0].astype(np.float32)
        if self.out["dtype"] != np.float32:                 # 양자화 출력 → 0~1
            scale, zero = self.out.get("quantization", (1 / 255, 0))
            y = (y - zero) * (scale or 1 / 255)
        i = int(np.argmax(y))
        return (self.labels[i] if i < len(self.labels) else str(i)), float(y[i])
