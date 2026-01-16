#inspection.py
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import google.generativeai as genai
import json
import time
from PIL import Image as PILImage
import subprocess
import os
from dotenv import load_dotenv
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent
ENV_PATH = Path("/home/teamone/ros2_ws3/src/.env")
load_dotenv(ENV_PATH)

GEMINI_API_KEY = os.getenv("GOOGLE_API_KEY")

CANDIDATE_ITEMS = ["Buttering", "Fire Extinguisher", "Human", "crown_sando"]

class InspectionBot(Node):
    def __init__(self):
        super().__init__('inspection_bot')
        
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        genai.configure(api_key=GEMINI_API_KEY)
        self.model = genai.GenerativeModel('gemini-3-flash-preview')
        
        self.bridge = CvBridge()
        self.latest_cv_image = None
        
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, qos_profile_sensor_data)
        
        self.create_subscription(
            String, '/arrival_status', self.arrival_callback, 10)
        
        print(f"--- AI Inspector Ready ---\n식별 가능 목록: {CANDIDATE_ITEMS}")

    def image_callback(self, msg):
        try:
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def arrival_callback(self, msg):
        if "arrived" in msg.data:
            print("\n도착 확인! 식별을 시작합니다.")
            self.run_ai_identification()

    def capture_image(self):
        if self.latest_cv_image is not None:
            rgb_image = cv2.cvtColor(self.latest_cv_image, cv2.COLOR_BGR2RGB)
            return PILImage.fromarray(rgb_image)
        return None
    
    def show_result_in_terminal(self, result):
        # 1. 출력할 메시지 포맷팅
        status_icon = '🟢 정상' if result['is_normal'] else '🔴 불량'
        message = f"""
        ========================================
                  [AI INSPECTION RESULT]
        ========================================
        TARGET   : {result['detected_item']}
        STATUS   : {status_icon}
        REASON   : {result['reasoning']}
        ========================================
        """
        
        # 2. 쉘 명령어 작성 (따옴표 이스케이프 처리)
        safe_msg = message.replace('"', '\\"').replace("'", "'")
        
        # gnome-terminal을 열고 echo로 출력 후, 엔터키 입력 대기(read)
        cmd = f'gnome-terminal -- bash -c "echo \\"{safe_msg}\\"; echo; read -p \\"Press Enter to close...\\""'
        
        try:
            subprocess.run(cmd, shell=True)
        except Exception as e:
            print(f"새 터미널 열기 실패: {e}")
            # 실패 시 기존 터미널에라도 출력
            print(message)

    def run_ai_identification(self):
        time.sleep(1.0)
        pil_img = self.capture_image()

        if pil_img:
            if self.latest_cv_image is not None:
                
                print("이미지를 분석 중입니다...")
                result = self.analyze_image(pil_img)

                self.show_result_in_terminal(result)
                    
                print(f"\n📋 [검사 결과 보고서]")
                print(f" - 대상: {result['detected_item']}")
                print(f" - 상태 판정: {'🟢 정상' if result['is_normal'] else '🔴 불량'}")
                print(f" - 근거: {result['reasoning']}\n")    
    
            
        else:
            print("❌ 카메라 오류")

    def analyze_image(self, image):
        prompt = f"""
        You are a robot vision system.
        Look at this image and identify the main object.
        
        1. Choose the object name from this list: {CANDIDATE_ITEMS}.
        2. If none of them match, set "detected_item" to "Unknown".
        3. Determine if the object is in a "Normal" state (upright, intact) or "Abnormal" (fallen, damaged).

        Output JSON format:
        {{
            "detected_item": "String (Name from the list)",
            "is_normal": boolean,
            "reasoning": "Explain why you identified it as such in 1 sentence."
        }}
        """
        try:
            response = self.model.generate_content([prompt, image])
            text = response.text.strip().replace('```json', '').replace('```', '')
            return json.loads(text)
        except Exception as e:
            return {"detected_item": "Error", "is_normal": False, "reasoning": str(e)}

def main():
    rclpy.init()
    bot = InspectionBot()
    rclpy.spin(bot)
    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()