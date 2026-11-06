# 표지판 모델

Teachable Machine(이미지 프로젝트)에서 학습한 표지판 모델을 여기에 넣고 `colcon build` 한다.
내보낸 파일 이름을 그대로 쓴다 (2026-11-06부터).

| 파일 | 내용 |
|---|---|
| `model_unquant.tflite` | TensorFlow Lite로 내보낸 부동소수 모델 (입력 224×224, 출력 클래스 5개) |
| `labels.txt` | 같이 받은 클래스 이름 — `HOME`, `BOTTLE`, `BOX`, `LOAD`, `none` |

- 파일 이름은 `params.yaml`의 `classifier_node` → `sign_model`, `sign_labels`
- 클래스 이름 → 표지판은 `label_map`으로 바꾼다 (대소문자 무시)

  | 모델 클래스 | 표지판 | 지점 |
  |---|---|---|
  | `LOAD` | `sign_work` | 상차 지점 |
  | `BOTTLE` | `sign_drop_a` | 원(병) 하차 지점 |
  | `BOX` | `sign_drop_b` | 사각(상자) 하차 지점 |
  | `HOME` | `sign_park` | 대기장소 |
  | `none` | `background` | 표지판 없음 — 확정하지 않음 |

- 다시 학습해서 클래스 이름이 바뀌면 `label_map`도 고친다
- 모델이 없어도 `classifier_node`는 뜨고, 표지판 확인 때 `E_SIGN`(초기 설정 중이면 `E_MAP`)으로 멈춘다 (도착 시험은 "표지판 못 읽음"으로 알림)
