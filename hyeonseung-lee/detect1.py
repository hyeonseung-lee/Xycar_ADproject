#!/usr/bin/env python

import cv2, time
import numpy as np

cap = cv2.VideoCapture('2.avi')


#################### initialize ####################
value_threshold = 180
#이미지 이진화에 이용할 명도 하한.
#----> 실험을 토대로 값을 설정.

image_width = 640
# frame of video file : width*height = 640*480
scan_width, scan_height = 160, 60
scan_width_L, scan_width_R = scan_width, scan_width
#차선 검출을 위하여 검사할 영역의 가로 세로 너비의 크기
#가로 너비가 클 경우에는 중앙선 부근에 반사된 빛을 인식하는 경우가 생긴다.
#----> 실험을 토대로 값을 설정.
scanW_control_unit = 5
lmid, rmid = scan_width_L, image_width - scan_width_R
#왼쪽, 오른쪽 검사를 진행할 영역의 안쪽 끝
area_width, area_height = 20, 50
#흰 픽셀의 개수를 검사할 영역의 가로와 세로 크기 설정.
#----> 실험을 토대로 값을 설정.
roi_vertical_pos = 300
# ROI설정을 위한 세로좌표 (=감지 영역의 위쪽 끝, 가로줄)
# ROI(= Region Of Interest) : 관심영역
row_begin = (scan_height - area_height) // 2
row_end = row_begin + area_height


white_pixel_percent_origin = 0.1
white_pixel_percent = white_pixel_percent_origin
wpp_control_unit = 0.05    # 차선이 인식되지 않을 대 감소시킬 퍼센테이지의 단위
pixel_cnt_threshold = white_pixel_percent * area_width * area_height
                        #검사 영역을 차선으로 판단하는
                        #흰 픽셀 비율의 하한
                        #----> 실험을 토대로 값을 설정.


while True:
    ret, frame = cap.read()

    # 종료조건
    if not ret:
        break   #영상이 끝나면 종료
    if cv2.waitKey(1) & 0xFF == 27:
        break   #ESC키가 눌리면 종료

    roi = frame[roi_vertical_pos:roi_vertical_pos + scan_height, :]

    #ROI설정 아까 지정한 부분부터 일정 크기의 세로 띠 모양
    frame = cv2.rectangle(frame, (0, roi_vertical_pos),
        (image_width - 1, roi_vertical_pos + scan_height),
        (255, 0, 0), 3)
        #설정된 ROI의 둘레에 파란색 사각형을 그림. 눈으로 확인하기 위함.
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    #roi로 끊어낸 부분을 hsv로 표현

    lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
    ubound = np.array([131, 255, 255], dtype=np.uint8)
    # 이미지 이진화를 위한 색상과 명도 범위 지정

    bin = cv2.inRange(hsv, lbound, ubound)      #이진화된 이미지
                        # ---> self.mask in linedetector.py
    view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)#이것을 다시 컬러 이미지로
                                                #녹색 사각형을 표시하기 위함

    left, right = -1, -1    #초기값 설정

    ###### 자율주행 스튜디오의 바닥이 차선 바깥쪽이 더 밝아 반사광의 양이 많다. 안쪽에서 바깥쪽으로 차선검출을 진행하도록 한다. ######

    # left (0 ~ scan_width, 450-scan_height ~ 450) 차선검출
    for l in range(lmid, area_width, -1):   # 중앙선에서 왼쪽 방향으로 차선검출 <- |
            # 검출 지역의 x 축 평행 길이, y 축 평행 길이
        area = bin[row_begin:row_end, l - area_width:l] 
        if cv2.countNonZero(area) > pixel_cnt_threshold:
            left = l
            break
                                                    # 위에서 설정한 하한값과 비교
    # right (640-scan_width ~ 640, 450-scan_height ~ 450) 차선검출
    for r in range(rmid, image_width - area_width):     # 중앙선에서 오른쪽 방향으로 차선검출 | ->
        area = bin[row_begin:row_end, r:r + area_width]
        if cv2.countNonZero(area) > pixel_cnt_threshold:
            right = r
            break

    if left != -1:  #왼쪽 차선이 검출되었을 때 ROI(잘라낸 이미지)에 녹색 사각형을 그림
        lsquare = cv2.rectangle(view,
                                (left - area_width, row_begin),
                                (left, row_end),
                                (0, 255, 0), 3)


        while white_pixel_percent < white_pixel_percent_origin:
            white_pixel_percent += wpp_control_unit
            print("white_pixel_percent : %f" % white_pixel_percent)


        while scan_width_L > 150:
            scan_width_L -= scanW_control_unit
            print("scan_width_L : %d" % scan_width)

    else:
        print("\nLost left line")
        if white_pixel_percent > 0.001: white_pixel_percent -= wpp_control_unit
        print("white_pixel_percent : %f" % white_pixel_percent)

        if scan_width_L <= 320: scan_width_L += scanW_control_unit
        print("scan_width_L : %d" % scan_width_L)


    if right != -1: #오른쪽 차선이 검출되었을 때 ROI(잘라낸 이미지)에 녹색 사각형을 그림
        rsquare = cv2.rectangle(view,
                                (right, row_begin),
                                (right + area_width, row_end),
                                (0, 255, 0), 3)

        while white_pixel_percent < white_pixel_percent_origin :
            white_pixel_percent += wpp_control_unit
            print("white_pixel_percent : %f" % white_pixel_percent)

        while scan_width_R > 150:
            scan_width_R -= scanW_control_unit
            print("scan_width_R : %d" % scan_width_R)

    else:
        print("\nLost right line")
        if white_pixel_percent > 0.001: white_pixel_percent -= wpp_control_unit
        print("white_pixel_percent : %f" % white_pixel_percent)

        if scan_width_R <= 320: scan_width_R += scanW_control_unit
        print("scan_width_R : %d" % scan_width_R)

    cv2.imshow("origin", frame)     #origin 윈도우에 카메라 영상 + ROI파란사각형 표시
    cv2.imshow("view", view)        #view라는 윈도우에 ROI+검출시 녹색 사각형 표시

    time.sleep(0.1)

cap.release()
cv2.destroyAllWindows()
