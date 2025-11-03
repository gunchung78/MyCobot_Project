# MyCobot_Project

# mycobot_ros2 (Humble, Ubuntu 22.04, VirtualBox) — 빠른 실행 가이드

myCobot 320을 **ROS 2 Humble** 환경에서 빠르게 구동하기 위한 자동 설정 스크립트와 실행 방법입니다.

---

## 1) 설치 스크립트 실행

### 1-1) 실행 권한

```bash
chmod +x setup_mycobot_ros2.sh
```

### 1-2) 기본 설치

```bash
./setup_mycobot_ros2.sh
```

### 1-3) MoveIt까지 함께 설치(선택)

```bash
./setup_mycobot_ros2.sh --with-moveit
```

> 스크립트 실행이 끝나면 새로운 터미널을 열거나 아래를 실행하세요:

```bash
source ~/mycobot_ros2/install/setup.bash
```

---

## 2) VirtualBox에서 myCobot 320 포트 연결

1. **VirtualBox Extension Pack** 설치(호스트에).
2. VM 전원 **끄기** → **설정 → USB**에서 **USB 3.0(xHCI)** 활성화.
3. **USB 필터 추가**로 myCobot 장치 선택(호스트 장치 관리자/디바이스 목록에서 모델 확인).
4. VM **부팅 후** myCobot를 다시 연결(또는 메뉴 **Devices → USB → 장치 체크**).

장치가 게스트(Ubuntu)로 제대로 패스스루되면 아래처럼 포트가 보입니다.

```bash
ls -a /dev/tty* | grep ACM0
```

**예상 출력**

```
/dev/ttyACM0
```

> 포트 이름은 환경에 따라 `/dev/ttyUSB0`로 보일 수도 있습니다.

---

## 3) (선택) 포트 권한 & 동작 확인

시리얼 접근 권한이 없으면 아래를 한 번만 설정하세요.

```bash
sudo usermod -a -G dialout $USER
newgrp dialout
```

간단 확인:

```bash
ls -l /dev/ttyACM0
```

---

## 4) 간단 실행 예시

GUI로 실기 제어(포트/보레이트는 장치에 맞게 조정):

```bash
ros2 run mycobot_320 simple_gui --ros-args -p port:=/dev/ttyACM0 -p baud:=115200
```

RViz에서 모델만 확인(옵션):

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui &
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix mycobot_320)/share/mycobot_320/urdf/mycobot_320.urdf.xacro)"
```

---

## 5) 트러블슈팅(짧게)

* **GUI/노드가 포트를 못 찾음** → VirtualBox USB 3.0 + USB 필터, `/dev/ttyACM0` 존재 확인
* **권한 오류** → `dialout` 그룹 추가 후 다시 연결
* **GUI가 반응 없음** → GUI 실행 시 `port`, `baud` 파라미터 지정, 속도(speed) 0 아닌지 확인

---

필요하면 README 최상단에 프로젝트 소개/라이선스/스크린샷 섹션만 추가하면 바로 공개 가능합니다.
