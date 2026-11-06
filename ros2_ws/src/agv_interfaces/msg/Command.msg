# bt_bridge_node → mission_node — 블루투스 명령
string  kind            # SETUP / START / STOP / RESUME / MANUAL_ON / MANUAL_OFF / MOVE
uint8   circle          # START: 원 개수 (0~9)
uint8   square          # START: 사각형 개수 (0~9)
float32 linear          # MOVE: 선속도 (-1.0 ~ 1.0)
float32 angular         # MOVE: 회전 (-1.0 ~ 1.0)
