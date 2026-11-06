# mission_node → bt_bridge_node (2Hz + 바뀔 때) — 블루투스 STATUS·MAP
string   state          # 상태 머신 상태 (IDLE, NAVIGATE, APPROACH, CHECK_SIGN, LOAD, UNLOAD, UTURN, PARKED, ...)
string   mode           # NONE / SETUP / WORK
uint8    done           # 작업: 하차한 개수 / 초기 설정: 확인한 지점 수
uint8    total          # 작업: 입력한 개수(원 + 사각) / 초기 설정: 4
string   cargo          # 실은 모양 NONE / CIRCLE / SQUARE
uint8    load           # 실은 개수 (7세그먼트와 같음)
string   target         # 작업: WORK / DROP_A / DROP_B / PARK, 초기 설정: 탐색 중인 색
string[] map            # 트랙 매핑 "WORK=blue" 4개, 모르면 "WORK=-"
string   message        # 오류·알림 문구
