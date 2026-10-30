"""Teachable Machine TFLite 모델을 Pi에서 LiteRT로 실행 — 추론 시간 측정 + 사진 분류 (관문 G1).

    python3 litert_check.py model_unquant.tflite labels.txt                 # 추론 시간만 (50회)
    python3 litert_check.py model_unquant.tflite labels.txt -i test/*.jpg   # 사진마다 분류 + 정답률

정답률: 사진 파일 이름이나 폴더 이름에 라벨(circle, square, background …)이 들어 있으면 정답으로 비교한다.
전처리는 Teachable Machine과 같게: 가운데 정사각 크롭 → 224×224 → RGB → (부동소수 모델) 픽셀/127.5 − 1
"""
import argparse
import os
import statistics
import time

import numpy as np


def load_interpreter(path):
    try:
        from ai_edge_litert.interpreter import Interpreter
        runtime = "ai-edge-litert"
    except ImportError:
        try:
            from tflite_runtime.interpreter import Interpreter
            runtime = "tflite-runtime"
        except ImportError:
            from tensorflow.lite import Interpreter
            runtime = "tensorflow"
    it = Interpreter(model_path=path)
    it.allocate_tensors()
    return it, runtime


def load_labels(path):
    labels = []
    for line in open(path, encoding="utf-8"):
        parts = line.strip().split(maxsplit=1)   # TM 형식: "0 circle"
        if parts:
            labels.append(parts[-1])
    return labels


def preprocess(path, size, dtype):
    from PIL import Image
    img = Image.open(path).convert("RGB")
    w, h = img.size
    s = min(w, h)
    img = img.crop(((w - s) // 2, (h - s) // 2, (w + s) // 2, (h + s) // 2)).resize((size, size))
    x = np.asarray(img)
    if dtype == np.float32:
        x = x.astype(np.float32) / 127.5 - 1.0
    else:
        x = x.astype(dtype)
    return x[None, ...]


def main():
    ap = argparse.ArgumentParser(description="TFLite 추론 시간·분류 점검")
    ap.add_argument("model")
    ap.add_argument("labels")
    ap.add_argument("-i", "--images", nargs="*", default=[], help="분류할 사진")
    ap.add_argument("-n", "--runs", type=int, default=50, help="시간 측정 횟수")
    args = ap.parse_args()

    it, runtime = load_interpreter(args.model)
    inp, out = it.get_input_details()[0], it.get_output_details()[0]
    size = int(inp["shape"][1])
    labels = load_labels(args.labels)
    print(f"런타임 {runtime} / 입력 {[int(v) for v in inp['shape']]} {inp['dtype'].__name__} / 라벨 {labels}")

    # 추론 시간 (처음 3회는 준비 실행이라 제외)
    dummy = np.zeros(inp["shape"], dtype=inp["dtype"])
    times = []
    for k in range(args.runs + 3):
        it.set_tensor(inp["index"], dummy)
        t0 = time.perf_counter()
        it.invoke()
        if k >= 3:
            times.append((time.perf_counter() - t0) * 1000)
    times.sort()
    p95 = times[int(len(times) * 0.95) - 1]
    verdict = "통과" if p95 <= 200 else "초과"
    print(f"추론 시간 {args.runs}회: 평균 {statistics.mean(times):.1f} ms / 95% {p95:.1f} ms / 최대 {times[-1]:.1f} ms"
          f" → 목표 200ms 이하 {verdict}")

    if not args.images:
        return
    right = total = 0
    for path in args.images:
        it.set_tensor(inp["index"], preprocess(path, size, inp["dtype"]))
        it.invoke()
        y = it.get_tensor(out["index"])[0].astype(np.float32)
        if out["dtype"] != np.float32:
            y = y / 255.0
        k = int(np.argmax(y))
        guess = labels[k] if k < len(labels) else str(k)
        truth = next((lb for lb in labels if lb.lower() in path.replace("\\", "/").lower()), None)
        mark = ""
        if truth is not None:
            total += 1
            right += guess == truth
            mark = "O" if guess == truth else f"X (정답 {truth})"
        name = os.path.join(os.path.basename(os.path.dirname(path)), os.path.basename(path))
        print(f"{name:30s} {guess:12s} {y[k]:.2f} {mark}")
    if total:
        print(f"정답률 {right}/{total} = {right / total * 100:.1f}% → 목표 95% 이상 {'통과' if right / total >= 0.95 else '미달'}")


if __name__ == "__main__":
    main()
