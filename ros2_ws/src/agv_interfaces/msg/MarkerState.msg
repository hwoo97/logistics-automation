# marker_node → drive_node·bt_bridge_node — 이번 프레임에 보이는 색 스티커 (세 배열은 같은 길이)
string[]  colors        # red / yellow / green / blue
string[]  sides         # left / center / right — center = 앞으로 이어지는 라인 위(직진), left/right = 옆 갈래 위 (2026-11-06)
float32[] y             # 화면 높이 (320×240 기준) — 클수록 차에 가까움
