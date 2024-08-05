## aruco_calbirate.launch ##
- aruco_type: aruco marker type
- aruco_save_dir: 마커들의 transform(x,y,z,x,y,z,w)를 저장하는 경로
- aruco_main_marker_id: main marker, 중심이 되는 마커로 다른 마커들의 위치는 main 마커를 기준으로 위치 관계를 나타냄

## arucode_node.launch ##
- aruco_node.py 파일에 calculate_camera_to_base_transform 함수 추가(로봇 1에 대해)
- base_frame parameter 추가, default False
- base_frame True로 설정 시 base 좌표계로 변환되어 출력
- End-effector to base transformation matrix 수정 필요
