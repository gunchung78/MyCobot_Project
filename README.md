## 환경

* os : ubuntu 22.04
* ros version : ros2 humble
* python version : 3.10

---

## 핵심 정리

* **설치:** `chmod +x setup_mycobot_ros2.sh && ./setup_mycobot_ros2.sh`
* **VirtualBox 설정:** (게스트 전원 종료) → **USB 3.0** 선택 → **USB 필터**로 myCobot 장치 추가
* **장치 확인(게스트 내부):** `ls /dev/tty* | grep AMA0` (환경에 따라 `ACM0`/`USB0`일 수 있음)
* **실행 예시:**

  ```bash
  ros2 launch mycobot_320 simple_gui.launch.py
  ```

---

## 1) 설치 스크립트 실행

1. 파일 저장(예: 홈 폴더):

   ```bash
   nano ~/setup_mycobot_ros2.sh   # 내용 붙여넣기 후 저장
   chmod +x ~/setup_mycobot_ros2.sh
   ```
2. 실행:

   ```bash
   ./setup_mycobot_ros2.sh
   ```
3. 완료 후 안내에 따라 새 터미널에서:

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/MyCobot_Project/mycobot_ros2/install/setup.bash
   ```

---

## 2) VirtualBox에서 myCobot USB 연결 설정

> **중요:** 이 절차는 **게스트(가상 머신) 전원을 완전히 끈 상태**에서 진행합니다.

1. **VirtualBox Extension Pack 설치(호스트)**
   USB 3.0 지원을 위해 권장됩니다. (VirtualBox 메뉴의 *Preferences → Extensions*에서 확인)

2. **게스트 설정 열기 → USB**

   * **USB Controller 활성화:** **USB 3.0 (xHCI) Controller** 선택
---

## 3) 게스트(우분투)에서 포트 확인 & 권한

**시리얼 장치 노출 확인**
   사용 환경에 따라 myCobot은 아래 중 하나로 보일 수 있습니다.

   * `/dev/ttyAMA0` (요청하신 확인 명령)
   * `/dev/ttyACM0`
   * `/dev/ttyUSB0`

   ```bash
   ls /dev/tty* | grep AMA0
   ```
---

## 4) PLC 통신 관련 패키지 설치

1. **Modbus 통신을 위한 pymodbus 설치**
   pip install pymodbus

2. **OPC UA 통신을 위한 asyncua 설치**
   pip install asyncua

## 5) 실행 예시

ros2 launch mycobot_320 simple_gui.launch.py
---


