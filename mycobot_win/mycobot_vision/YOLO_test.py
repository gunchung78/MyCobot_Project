import cv2
from ultralytics import YOLO
import yaml

# --- 설정값 ---
# 1. 모델 파일 경로: 학습이 완료된 YOLO 모델 파일 경로 (예: runs/detect/train/weights/best.pt)
MODEL_PATH = 'C:/Dev/KAIROS_Project_2/AI_Cordinate/YOLO/runs/detect/train/weights/best.pt' 

# 2. 카메라 인덱스: 0번은 보통 노트북 내장 카메라, 1번은 외부 연결된 첫 번째 카메라입니다.
# 0 -> robot arm
# 2 -> daiso
CAMERA_INDEX = 0

# 3. YAML 파일에서 클래스 이름 로드 (시각화를 위해)
# 실제 학습에 사용된 YAML 파일을 읽어 클래스 목록을 가져옵니다.
YAML_PATH = 'C:/Dev/KAIROS_Project_2/AI_Cordinate/YOLO/team/custom_data.yaml'

try:
    with open(YAML_PATH, 'r') as f:
        data_yaml = yaml.safe_load(f)
        CLASS_NAMES = data_yaml['names']
        print(f"✅ 클래스 이름 로드 성공: {CLASS_NAMES}")
except FileNotFoundError:
    print(f"🚨 오류: YAML 파일을 찾을 수 없습니다. 경로를 확인하세요: {YAML_PATH}")
    exit()

# --- 모델 로드 ---
try:
    # 학습된 YOLO 모델을 로드합니다.
    model = YOLO(MODEL_PATH)
    print("✅ YOLO 모델 로드 성공.")
except Exception as e:
    print(f"🚨 오류: YOLO 모델 로드 실패. 경로를 확인하거나 파일이 손상되지 않았는지 확인하세요. 오류: {e}")
    exit()

def main():
    # --- 카메라 설정 ---
    # 카메라 객체 생성 (CAMERA_INDEX에 해당하는 카메라를 엽니다)
    cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        print(f"🚨 오류: 카메라 인덱스 {CAMERA_INDEX}번을 열 수 없습니다. 카메라 연결 상태를 확인하세요.")
        exit()

    print(f"🚀 실시간 탐지 시작: 카메라 {CAMERA_INDEX}번 연결됨. (종료: 'q' 키)")

    # --- 메인 루프: 실시간 탐지 ---
    while True:
        # 1. 프레임 읽기
        ret, frame = cap.read()
        
        if not ret:
            print("경고: 프레임을 읽을 수 없습니다. (카메라 연결 끊김?)")
            break
        
        # 2. YOLO 모델로 탐지 수행
        # device=0 (첫 번째 GPU)를 사용하도록 명시적으로 설정합니다.
        results = model(frame, device=0, verbose=False) 
        
        # 3. 탐지 결과를 프레임에 시각화
        # results[0].plot() 함수는 바운딩 박스와 라벨을 자동으로 그려줍니다.
        annotated_frame = results[0].plot()
        
        # 4. 화면에 표시
        cv2.imshow("YOLO Real-Time Detection (Press 'q' to exit)", annotated_frame)
        
        # 'q' 키를 누르면 루프 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # --- 종료 처리 ---
    cap.release()
    cv2.destroyAllWindows()
    print("👋 탐지 테스트가 종료되었습니다.")

if __name__ == "__main__":
    main()
