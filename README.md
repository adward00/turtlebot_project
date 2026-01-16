# turtlebot_project_my work

1. DWB알고리즘 연구 및 구현 실패

2. yolov8을 이용해 칠성사이다와 코카콜라를 인식해 물품 배치 잘못되었음을 알리는 기능 구현 (cocacola_yolov8폴더와 chilsung_yolov8폴더) master branches에 업로드 해놓음

3. Theta* 알고리즘 구현 (rpp_test.py)

전역 경로 계획(Global Path Planning) -> theta*

-> A*의 변형

-> 부모 노드와 Line-of-Sight(직선 가시성) 이 있으면 중간 노드를 생략

-> 결과 경로가 지그재그가 아닌 직선 위주
-> Theta*는
Grid 기반 환경에서 Line-of-Sight를 이용해
A*의 격자 제약을 제거한 Any-angle 전역 경로 계획 알고리즘이다.


지역 경로 추종(Local Path Following) -> Adaptive Pure Pursuit
-> Lookahead distance 가변 조정

-> 곡률(curvature)에 따라 속도·조향 조절

-> 알고리즘의 뼈대는 Pure Pursuit 그대로이고
Lookahead, 속도, 반응 강도를 실시간으로 바꾸는 방식입니다.

4. 202호 강의실 맵 그리기 및 pgm편집기를 이용해 부분적으로 벽이 뚫려있는 곳을 갈 수 있는 길로 인식하지 못하게 맵 편집 (map_wall_202.pgm)




