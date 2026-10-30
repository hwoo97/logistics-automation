# AI 런타임 점검 — Teachable Machine 모델을 Pi에서 실행 (관문 G1)

> 목표: Teachable Machine으로 만든 화물 모델(원·사각형·배경)을 Pi 5 + Ubuntu 24.04(Python 3.12)에서 **LiteRT**로 실행하고, 인식률·1회 추론 시간을 잰다.
> 클래스·전처리·목표: [Teachable Machine 클래스](../../02_design/ai/tm_classes.md)

## 1. 모델 만들기 (PC 브라우저)
1. https://teachablemachine.withgoogle.com → **이미지 프로젝트 → 표준 이미지 모델**
2. 클래스 3개: `circle`, `square`, `background` (이름은 영문 그대로 — 스크립트가 라벨로 정답을 비교)
3. 각 클래스 **웹캠으로 200장 이상**: 거리(가깝게·중간·멀게), 좌우로 조금 어긋나게, 약간 기울여서. `background`는 빈 책상·벽·손 등
4. **모델 학습시키기** → 미리보기에서 대충 맞는지 확인
5. **모델 내보내기 → Tensorflow Lite → 부동소수점 → 다운로드** → `converted_tflite.zip` (`model_unquant.tflite`, `labels.txt`)

## 2. 시험 사진 (인식률용)
- 학습에 쓰지 않은 **새 사진**을 클래스마다 20장: 휴대폰으로 찍어도 됨
- 파일 이름이나 폴더에 라벨을 넣는다: `test/circle/01.jpg`, `test/square/01.jpg`, `test/background/01.jpg`

## 3. Pi에서 실행
```bash
pip install ai-edge-litert pillow --break-system-packages   # 막히면: python3 -m venv ~/venv-ai && ~/venv-ai/bin/pip install ai-edge-litert pillow
cd ~/logistics-automation/03_setup/ai
python3 litert_check.py ~/model/model_unquant.tflite ~/model/labels.txt                       # 추론 시간
python3 litert_check.py ~/model/model_unquant.tflite ~/model/labels.txt -i ~/test/*/*.jpg     # 인식률
```
- 모델·사진은 레포에 넣지 않는다 (용량·사진) — USB나 scp로 Pi에 복사

## 4. 통과 기준
| # | 항목 | 기준 |
|---|------|------|
| 1 | 런타임 설치 | `ai-edge-litert` import 성공 (안 되면 `tflite-runtime` → 대체 런타임 검토) |
| 2 | 추론 시간 | 1회 95% 200ms 이하 |
| 3 | 인식률 | 시험 사진 95% 이상 |

## 5. 결과 기록
| 날짜 | 담당 | 항목 | 결과 | 비고 |
|------|------|------|------|------|
