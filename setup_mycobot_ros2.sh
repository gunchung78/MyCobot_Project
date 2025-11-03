#!/usr/bin/env bash
# setup_mycobot_ros2.sh
# Ubuntu 22.04 + ROS2 Humble 에서 mycobot_ros2(320 포함) 최소 실행 환경 자동 세팅
# 옵션: --with-moveit  -> ros-humble-moveit 설치

set -euo pipefail

WITH_MOVEIT="false"
for arg in "$@"; do
  case "$arg" in
    --with-moveit) WITH_MOVEIT="true" ;;
    *) echo "Unknown option: $arg"; exit 1 ;;
  esac
done

echo "=== [1/7] 시스템 패키지 업데이트 ==="
sudo apt update

echo "=== [2/7] 필수 패키지 설치 ==="
sudo apt install -y \
  python3-argcomplete \
  ros-humble-xacro \
  python3-colcon-common-extensions \
  ros-humble-joint-state-publisher-gui

if [ "$WITH_MOVEIT" = "true" ]; then
  echo "=== (옵션) MoveIt 설치 ==="
  sudo apt install -y ros-humble-moveit
else
  echo "=== MoveIt 설치 건너뜀 (원하면 --with-moveit 옵션으로 설치) ==="
fi

echo "=== [3/7] pymycobot 설치 ==="
# 시스템 파이썬에 설치 (ROS2 런치와 동일 파이썬 사용 보장)
sudo apt install -y python3-pip
pip install pymycobot

echo "=== [4/7] 워크스페이스 생성 ==="
WS="$HOME/MyCobot_Project/mycobot_ros2"
mkdir -p "$WS/src"

echo "=== [5/7] 소스 클론 ==="
cd "$WS/src"
if [ ! -d "mycobot_ros2" ]; then
  git clone --depth 1 https://github.com/elephantrobotics/mycobot_ros2.git
else
  echo "이미 mycobot_ros2가 존재합니다. 업데이트를 원하면 수동으로 git pull 하세요."
fi

echo "=== [6/7] 빌드 ==="
cd "$WS"
# python 스크립트 수정 시 재빌드 피하려면 --symlink-install 유지
colcon build --symlink-install

echo "=== [7/7] 환경 설정 ==="
# 현재 세션
# shellcheck disable=SC1090
source "$WS/install/setup.bash"

# 선택적으로 bashrc에 추가
BASHRC_LINE="source \$HOME/MyCobot_Project/mycobot_ros2/install/setup.bash"
if ! grep -Fxq "$BASHRC_LINE" "$HOME/.bashrc"; then
  echo "$BASHRC_LINE" >> "$HOME/.bashrc"
  echo "bashrc에 워크스페이스 설정을 추가했습니다. (새 터미널에서 자동 적용)"
else
  echo "bashrc에 이미 워크스페이스 설정이 존재합니다."
fi

echo
echo "✅ 완료!"
echo "다음 명령으로 환경 확인:"
echo "  source \$HOME/MyCobot_Project/mycobot_ros2/install/setup.bash"
echo "  ros2 pkg list | grep mycobot"
echo
echo "실기 테스트 전 권장:"
echo "  - USB 패스스루(가상머신): 장치가 /dev/ttyACM0 또는 /dev/ttyUSB0로 보여야 함"
echo "  - dialout 권한: sudo usermod -a -G dialout \$USER && newgrp dialout"
echo
echo "간단 실행 예시(포트/보레이트는 장치에 맞게):"
echo "  ros2 run mycobot_320 simple_gui --ros-args -p port:=/dev/ttyACM0 -p baud:=115200"
echo "  # RViz로 모델만 보고 싶으면:"
echo "  ros2 run joint_state_publisher_gui joint_state_publisher_gui &"
echo "  ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"\$(xacro \$(ros2 pkg prefix mycobot_320)/share/mycobot_320/urdf/mycobot_320.urdf.xacro)\""
