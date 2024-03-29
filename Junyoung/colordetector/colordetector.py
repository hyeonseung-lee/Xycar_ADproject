import cv2

# 색상 범위 설정
lower_orange = (10, 0, 100)
upper_orange = (30, 255, 255)

lower_green = (50, 0, 100)
upper_green = (70, 255, 255)

lower_red = (0, 200, 240)
upper_red = (10, 255, 255)

# 이미지 파일을 읽어온다
img = cv2.imread("rgb.png", cv2.IMREAD_COLOR)

# BGR to HSV 변환
img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# 색상 범위를 제한하여 mask 생성
img_mask1 = cv2.inRange(img_hsv, lower_orange, upper_orange)
img_mask2 = cv2.inRange(img_hsv, lower_green, upper_green)
img_mask3 = cv2.inRange(img_hsv, lower_red, upper_red)

# # 원본 이미지를 가지고 Object 추출 이미지로 생성
# img_result1 = cv2.bitwise_and(img, img, mask=img_mask1)
# img_result2 = cv2.bitwise_and(img, img, mask=img_mask2)
# img_result3 = cv2.bitwise_and(img, img, mask=img_mask3)

# 결과 이미지 생성
cv2.imshow('orange', img_mask1)
cv2.imshow('green', img_mask2)
cv2.imshow('red', img_mask3)

# 종료
cv2.waitKey(0)
cv2.destroyAllWindows()