# 표지판 모델

Teachable Machine(이미지 프로젝트)에서 학습한 표지판 모델을 여기에 넣고 `colcon build` 한다.

| 파일 | 내용 |
|---|---|
| `sign.tflite` | TensorFlow Lite로 내보낸 모델 (부동소수·양자화 모두 가능) |
| `labels_sign.txt` | 같이 받은 `labels.txt` 이름만 바꿈 — `sign_work`, `sign_drop_a`, `sign_drop_b`, `sign_park`, `background` |

- 클래스 이름은 위와 같아야 한다 ([tm_classes.md](../../../../02_design/ai/tm_classes.md))
- 모델이 없어도 `classifier_node`는 뜨고, 표지판 확인 때 `E_SIGN`(초기 설정 중이면 `E_MAP`)으로 멈춘다
