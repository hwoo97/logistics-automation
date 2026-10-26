# 팀 협업 안내

이 레포에서 함께 작업하는 방법. 자세한 개발 절차는 [개발 순서 및 절차](../02_design/development_plan.md), 주차별 할 일은 [주차별 실행 체크리스트](../02_design/weekly_checklist.md).

## 1. 처음 한 번
1. 초대 메일(또는 GitHub 알림)에서 **Accept invitation**
2. 코드 받기
   ```bash
   git clone https://github.com/hwoo97/logistics-automation.git
   cd logistics-automation
   git config user.name "이름"
   git config user.email "GitHub에 등록한 이메일"
   ```
3. 먼저 읽을 문서: [README](../README.md) → [개요서](../01_planning/overview.md) → [진행 프로세스](../PROCESS.md) → 맡은 흐름의 설계 문서

## 2. 작업 흐름
```bash
git switch main && git pull                       # 최신 받기
git switch -c feat/<패키지>-<내용>                  # 예: feat/vision-line-detector
# ... 작업, 테스트 ...
git add <파일>
git commit -m "[패키지] 내용"                       # 예: [agv_vision] 정지선 검출 추가
git push -u origin feat/<패키지>-<내용>
```
- GitHub에서 **Pull Request** 생성 → 팀원 1명 확인 → main에 병합
- 공용 파일(`agv_protocol/types.py`, `agv_interfaces`, `config/params.yaml`) 변경은 흐름 D 담당 확인
- 병합 전 확인: `pytest` 통과, Pi에서 `colcon build` 통과, 테스트 기록(`04_unit/<번호_기능>/test.md`) 작성
- main에 직접 push하지 않는다

## 3. 브랜치·커밋 이름
| 종류 | 형식 | 예 |
|------|------|------|
| 기능 | `feat/<패키지>-<내용>` | `feat/bt-video-channel` |
| 수정 | `fix/<패키지>-<내용>` | `fix/hw-motor-direction` |
| 문서 | `docs/<내용>` | `docs/test-track-s3-result` |
| 커밋 메시지 | `[패키지 또는 폴더] 내용` | `[03_setup] 블루투스 시험 결과 추가` |

## 4. 어디에 무엇을
| 내용 | 위치 |
|------|------|
| 차량 코드 (ROS 2 패키지) | `ros2_ws/src/` — [소프트웨어 구조 설계](../02_design/software/software_architecture.md)의 패키지·모듈 이름을 따른다 |
| 노트북 앱 | `laptop_app/` |
| 준비 단계 기록·점검 스크립트 | `03_setup/` |
| 단위 기능 테스트 기록 | `04_unit/<번호_기능>/test.md` |
| 설계 변경 | 해당 설계 문서를 같은 PR에서 수정 |

## 5. 올리지 않는 것
- 시험 사진, 학습 이미지(`04_unit/08_ai_vision/dataset/`), rosbag, 빌드 결과 — `.gitignore`에 등록됨
- 개인정보, 비밀번호·토큰, 교재 원문(사진·긴 인용) — **Public 레포**이므로 주의

## 6. Raspberry Pi
- Pi에서는 코드를 수정하지 않고 `git pull` → `colcon build --symlink-install`만 한다
- 급한 수정도 노트북에서 커밋·push 후 Pi에서 pull
