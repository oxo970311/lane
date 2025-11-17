🚗 Autonomous Lane Following & Object-Aware Driving Project
ROS2 + OpenCV + Python 기반 TurtleBot3 자율주행 시스템

본 프로젝트는 라즈베리파이(RPi) 와 랩탑(Off-board 연산) 을 결합하여
실시간 차선 인식 + 조향 제어 + 주행 상태 관리를 수행하는 ROS2 패키지입니다.

전체 구조는 다음 3개의 핵심 노드로 구성됩니다:

cam_pub – 카메라 영상 수집 및 퍼블리시

off_board – 차선 인식 / 원근 변환 / 조향각 계산

motor_control (또는 motor_sub) – 조향·속도 제어 및 안정적 주행

아래에서 각 노드의 역할과 동작 방식을 자세히 설명합니다.

📦 1. cam_pub — 카메라 퍼블리셔 노드

라즈베리파이 카메라 또는 USB 웹캠에서 프레임을 받아
ROS2 /image_raw 토픽으로 퍼블리시하는 역할을 담당합니다.

🔧 기능

카메라 초기화 (해상도, FPS 설정)

OpenCV → ROS Image 메시지 변환 (cv_bridge)

오프보드 PC로 프레임 송신

네트워크 환경에서도 안정적 전송을 위한 FPS 제한

📤 Publish
Topic	Type	설명
/image_raw	sensor_msgs/Image	원본 카메라 영상
🧠 2. off_board — 차선 인식 & 조향각 계산 노드

가장 핵심 로직이 들어 있는 메인 Vision Processing 노드입니다.
카메라 프레임을 입력받아 다음을 수행합니다.

🟩 ① ROI 정의

노이즈를 최소화하기 위해 도로 하단 중심 부분만 사용.

🟦 ② 원근 변환 (Bird’s-Eye View)

cv2.getPerspectiveTransform()을 사용하여
실제 도로와 유사한 정사영 형태로 변환하여 차선 인식 안정성 증가.

⚪ ③ 전처리

Grayscale

GaussianBlur

Morphology (열림/닫힘)

Threshold / Canny

🔳 ④ Sliding Window 기반 차선 검출

히스토그램으로 차선의 시작점(left/right) 찾고
슬라이딩 윈도우 10개 스택으로 레인 곡률 및 위치 추정.

🎯 ⑤ 조향각 산출

두 가지 방식 중 상황에 따라 선택하도록 구성:

방식	설명
중앙선 오프셋(angle)	레인 중심과 화면 중앙의 x 오차
기울기 기반(deg)	윈도우 중심들의 기울기 변화량 기반

각 방식은 /angle, /deg, /deg_valid 형태로 송신.

📤 Publish
Topic	Type	설명
/angle	Float32	중앙선 offset 기반 조향값
/deg	Float32	Slope 기반 조향값
/deg_valid	Bool	deg 사용 가능 여부
⚙️ 3. motor_control — 주행 제어 노드

차선 인식 결과를 받아 실제 터틀봇 움직임을 제어하는 역할입니다.

🧮 입력

/angle: 기본 조향각

/deg: 고급 곡률 기반 조향각

/deg_valid: deg 사용 가능 여부

/speed_cmd: 속도 조절 (up/down)

/emergency_stop: 긴급정지

/lane_change_cmd: 차선 변경(옵션)

🚘 제어 로직

deg_valid = True → /deg 기반 조향

deg_valid = False → /angle 기반 조향

LPF(저역통과 필터) 로 조향각 안정화 → z_filtered

최소 회전 / 최대 회전 제한

정지·비상·재시작 상태 완전 분리

📤 Publish
Topic	Type	설명
/cmd_vel	geometry_msgs/Twist	최종 주행 명령
🔗 전체 데이터 흐름
   [cam_pub]
        |
        v
  /image_raw (RPi → Laptop)
        |
        v
  [off_board Vision]
   - ROI
   - BEV (Perspective)
   - Sliding Window
   - Angle / Deg 계산
        |
        v
   /angle, /deg, /deg_valid
        |
        v
  [motor_control]
   - 속도/각도 연산
   - LPF 안정화
   - emergency stop
        |
        v
   /cmd_vel → TurtleBot 주행

🎥 시연 예시 (추가 예정)

 차선 추종 영상

 차선 변경 (중앙선 인식 기반)

 장애물/표지판 감지 후 제어 (YOLO 연동 가능)

🧩 프로젝트 특징 요약
✔ 오프보드 방식으로 고성능 Vision 처리

라즈베리파이 CPU 부담 최소화 & 안정적 FPS 유지

✔ BEV + Sliding Window로 신뢰성 높은 차선 검출

약한 조명, 노이즈 많은 도로 환경에서도 robust

✔ 조향각 듀얼 방식(angle + deg)

straight + curve 모두 안정적으로 대응

✔ LPF 조향 보정으로 흔들림 최소
✔ 긴급정지 / 정지 / 재시작 상태 완전 분리

버벅임 없이 주행 → 정지 → 재시작 가능

📁 폴더 구조 예시
bot_ws/
└── bot_pkg/
    ├── cam_pub.py
    ├── off_board.py
    ├── motor_control.py
    ├── launch/
    │   └── bot_all.launch.py
    ├── resource/
    └── setup.py

🛠 사용 기술

ROS2 Humble

Python3 + rclpy

OpenCV

NumPy

cv_bridge

TurtleBot3 Burger

Raspberry Pi 4

Off-board Vision Processing 구조

YOLO(Optional)
