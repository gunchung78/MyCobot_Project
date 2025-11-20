#!/usr/bin/env bash
set -euo pipefail

echo "=== [1/6] 시스템 패키지 업데이트 ==="
sudo apt update
sudo apt install -y git curl

echo "=== [2/6] 필수 패키지 설치 ==="
sudo apt install -y \
  python3-argcomplete \
  ros-humble-xacro \
  python3-colcon-common-extensions \
  ros-humble-joint-state-publisher-gui \
  python3-pip python3-wheel python3-setuptools \
  ros-humble-moveit


echo "=== [3/6] pymycobot 설치 (setuptools 업그레이드 안 함) ==="
python3 -m pip install --user --upgrade pip wheel
python3 -m pip install --user --no-cache-dir pymycobot

echo "=== [4/6] 워크스페이스 생성 ==="
WS="$HOME/MyCobot_Project/mycobot_ros2"
mkdir -p "$WS/src"

echo "=== [5/6] 빌드 ==="
cd "$WS"
set +e
colcon build
RC=$?
set -e
if [ $RC -ne 0 ]; then
  echo "[INFO] symlink-install 실패 → 일반 빌드로 재시도"
  colcon build
fi

echo "=== [6/6] 환경 설정(.bashrc 추가) ==="
# ROS2 → overlay 순서가 중요
if ! grep -Fxq 'source /opt/ros/humble/setup.bash' "$HOME/.bashrc"; then
  echo 'source /opt/ros/humble/setup.bash' >> "$HOME/.bashrc"
fi
if ! grep -Fxq "source $WS/install/setup.bash" "$HOME/.bashrc"; then
  echo "source $WS/install/setup.bash" >> "$HOME/.bashrc"
fi

# 현재 세션 반영
# shellcheck disable=SC1090
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash || true
# shellcheck disable=SC1090
source "$WS/install/setup.bash"

echo
echo "✅ 완료!"
echo "새 터미널을 열거나, 아래를 실행하세요:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $WS/install/setup.bash"
echo
echo "패키지 확인:"
echo "  ros2 pkg list | grep mycobot"
echo
echo "실행 예시:"
echo "  ros2 run mycobot_320 simple_gui --ros-args -p port:=/dev/ttyACM0 -p baud:=115200"
