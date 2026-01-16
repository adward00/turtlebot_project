# 생성 파트 
import cv2 
import qrcode 

# QR코드에 담고 싶은 내용 정하기 
text = "http://192.168.0.27:5050/"

# 큐알코드를 만들기 위해 인스턴스 생성 
qr = qrcode.QRCode(
    version=1,   # 1~40 까지의 범위. 데이터 볼륨을 뜻함. 
    box_size=10, # 블록(점)의 크기 
    border=4 # 블록 당 여백 크기 
)

# 큐알코드의 데이터 정의하고 생성하기 
qr.add_data(text)
qr.make()

# 생성된 큐알코드 이미지로 확인하기 
img = qr.make_image(fill_color="black", back_color="white") 
img.save("qrcode_sample.png") # 주어진 이름으로 파일 생성

# OpenCV로 이미지 열어서 표시해보기 (선택)
img_cv = cv2.imread("qrcode_sample.png")
cv2.imshow("Generated QR", img_cv)
cv2.waitKey(0)
cv2.destroyAllWindows()