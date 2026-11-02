# stop_line_node → drive_node·bt_bridge_node
bool    detected        # 이번 프레임 판정 (drive_node는 이 값이 처음 나오면 일단 정지)
bool    confirmed       # 확인 프레임만큼 쌓여 확정
float32 angle           # 정지선 기울기(도), 0 = 수평, + = 오른쪽이 위
