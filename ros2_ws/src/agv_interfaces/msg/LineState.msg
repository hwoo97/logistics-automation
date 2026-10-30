# line_node → drive_node·bt_bridge_node (functional_design 사용자 정의 메시지)
float32 offset          # 라인 중심의 좌우 오차 (-1.0 왼쪽 ~ 1.0 오른쪽)
bool    line_found
string  event           # 확정 이벤트: NONE / STOP_LINE / JUNCTION_T / JUNCTION_LEFT / JUNCTION_RIGHT
                        #             CORNER_LEFT / CORNER_RIGHT / LINE_END
string  raw_event       # 이번 프레임 판정 (연속 프레임 확인 전) — 검출 값 조정용
