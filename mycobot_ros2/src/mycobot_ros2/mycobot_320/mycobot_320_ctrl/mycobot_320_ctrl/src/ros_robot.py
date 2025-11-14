# -*- coding: utf-8 -*-
from typing import Sequence, Optional


class ROS_Robot:
    def __init__(self, mode, C):
        """설정 로드 및 상태 초기화"""
        self.move_speed = int(C.MOVE_SPEED)
        self.gr_open = int(C.GRIPPER_OPEN_VAL)
        self.gr_close = int(C.GRIPPER_CLOSE_VAL)
        self.gr_speed = int(C.GRIPPER_SPEED)
        self.ANCHOR_PY = list(C.ANCHOR_PY)
        self.mode = mode
        self.result = []
        self.placeList = [0, 0, 0]

    def move_and_wait(self, mode: str, value: Sequence[float],
                      speed: Optional[int] = None, delay: float = 0.5, poll_sec: float = 0.1):
        """모션 명령을 플랜(result)에 적재 (실제 전송은 외부에서 수행)"""
        try:
            _ = speed or self.move_speed  # 인터페이스 호환 유지
            if mode in ("coords", "angles"):
                self.result.append([mode, list(value)])
            else:
                raise ValueError("mode must be 'coords' or 'angles'")
        except Exception as e:
            print(f"[WARN] move_and_wait 실패: {e}")

    def power_on(self, go_anchor: bool = True):
        """전원 온 시 초기자세 이동(옵션)"""
        try:
            if go_anchor:
                self.go_anchor()
        except Exception as e:
            print(f"[WARN] power_on 실패: {e}")

    def gripper_open(self):
        """그리퍼 열기 플랜 적재"""
        try:
            self.result.append(["gripper", True])
        except Exception as e:
            print(f"[WARN] gripper open 실패: {e}")

    def gripper_close(self):
        """그리퍼 닫기 플랜 적재"""
        try:
            self.result.append(["gripper", False])
        except Exception as e:
            print(f"[WARN] gripper close 실패: {e}")

    def place_box(self, val: str, idx: int = 0):
        """분류값에 따른 드롭존 이동 및 배치 플랜 적재"""
        z = 0
        try:
            if val in ('green', 'normal'):
                z = 145 + (idx * 24)
                self.move_and_wait("angles", [-10, 0, 78.95, -21, -87.36, -15])
                self.move_and_wait("coords", [-293.5, -25, z, -176, 0, 90], speed=10, delay=1.0)
                self.gripper_open()
                self.move_and_wait("angles", [-10, 0, 78.95, -21, -87.36, -15])

            elif val == 'blue':
                z = 155 + (idx * 25)
                self.move_and_wait("angles", [30, 0, 70.83, -16.08, -90, -150])
                self.move_and_wait("coords", [-219.2, -281.9, z, 179.41, -5, -100], speed=10, delay=1.0)
                self.gripper_open()
                self.move_and_wait("angles", [30, 0, 70.83, -16.08, -90, -150])

            elif val == 'red':
                z = 153 + (idx * 24)
                self.move_and_wait("angles", [136.66, 0, -55.98, 0, 109.51, -30])
                self.move_and_wait("coords", [-180, 240, z, -173.15, 0, 90], speed=10, delay=1.0)
                self.gripper_open()
                self.move_and_wait("angles", [136.66, 0, -55.98, 0, 109.51, -30])

            elif val == 'anomaly':
                self.move_and_wait("angles", [-80, -20, -65, -0, 90, -90])
                self.gripper_open()

            # 정의되지 않은 값은 아무 동작 없이 패스
            self.go_anchor()

        except Exception as e:
            print(f"[ERROR] place_box 실패: {e}")

    def go_anchor(self, speed: int = 20, delay: float = 0.2):
        """앵커(기준) 자세로 복귀 플랜 적재"""
        try:
            self.move_and_wait("angles", [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25], speed, delay)
            self.move_and_wait("angles", [0, 0, -80, -0, 90, -90], speed, delay)
        except Exception as e:
            print(f"[WARN] go_anchor 실패: {e}")

    def refresh_home(self, speed: int = 20, delay: float = 0.2):
        """앵커 근처 리프레시 동작 플랜 적재"""
        try:
            self.move_and_wait("angles", [0, 10, -80, -0, 90, -90], speed, delay)
            self.move_and_wait("angles", [0, 0, -80, -0, 90, -90], speed, delay)
        except Exception as e:
            print(f"[WARN] refresh_home 실패: {e}")

    def main_fow(self, x_t, y_t, rz_t, color, detected_type):
        """메인 피킹-분류-적재 플랜 생성"""
        self.result.clear()
        if rz_t is None:
            rz_t = self.ANCHOR_PY[5]

        try:
            # 입력 유효성 체크(모드별)
            if self.mode == "detect_only" and (color in (None, "white")):
                self.refresh_home()
                return list(self.result)

            if self.mode == "detect_and_classify" and (detected_type in (None, "unknown")):
                self.refresh_home()
                return list(self.result)

            # 접근 상부
            self.move_and_wait("angles", [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25])

            # 픽 포인트 접근
            self.move_and_wait(
                "coords",
                [x_t, y_t, self.ANCHOR_PY[2], self.ANCHOR_PY[3], self.ANCHOR_PY[4], rz_t],
                self.move_speed,
                0
            )

            # 집기 후 이탈
            self.gripper_close()
            self.move_and_wait("angles", [-6.94, 6.24, -55.19, -18.19, 81.03, -93.25])

            # 분류/적재
            if self.mode == "detect_only":
                if color == "red":
                    self.place_box("red", self.placeList[0]);   self.placeList[0] += 1
                    if(self.placeList[0] >= 3): self.placeList[0] = 0
                elif color == "green":
                    self.place_box("green", self.placeList[2]); self.placeList[2] += 1
                    if(self.placeList[2] >= 3): self.placeList[2] = 0
                elif color == "blue":
                    self.place_box("blue", self.placeList[1]);  self.placeList[1] += 1
                    if(self.placeList[1] >= 3): self.placeList[1] = 0

            elif self.mode == "detect_and_classify":
                if detected_type == "anomaly":
                    self.place_box("anomaly")
                elif detected_type == "normal":
                    self.place_box("normal", self.placeList[2]); self.placeList[2] += 1
                    if(self.placeList[2] >= 3): self.placeList[2] = 0

            return list(self.result)

        except Exception as e:
            print(f"[ERROR] 로봇 명령 실패: {e}")
            return list(self.result)

if __name__ == "__main__":
    from config_loader import load_config 
    C = load_config("../config.json")

    # 샘플 입력값
    mode = "detect_only"                 # 1: color 기반, 0: yolo 기반
    x_t, y_t, rz_t = -230.0, -50.0, 0.0
    color = "green"
    detected_type = "normal"

    # 인스턴스 생성
    bot = ROS_Robot(mode, C)
    print("\n=== Test Result * 3 ===")
    for i in range(3):
    # 메인 플로우 실행
        out = bot.main_fow(x_t, y_t, rz_t, color, detected_type)
        # 결과 출력
        print(i)
        for step in out:
            print(step)