########Readme#######
# 這個程式是利用 pyfirmata 跟 arduino 通訊
# 1. 確定python環境有 pyfirmata，沒有的話要pip install pyfirmata
# 2. 將 arduino 用usb連接到電腦
# 3. 在 arduino IDE 裡開啟 檔案->範例-> Firmata -> Standard Firmata
# 4. 將StandardFirmata檔案上傳到arduino
# 5. 要檢查通訊埠，在工具->序列埠裡察看
# 6. 把"下面arduino前置"的 port變數改成通訊的序列埠名稱
# *如果跑不動可能還要 pip install serial

from pyfirmata import Arduino, SERVO, util
import time
import cv2
import numpy as np
import math

#######arduino前置########

port = ('/dev/cu.usbserial-1410')
board = Arduino(port)
time.sleep(5)

def setServoAngle(pin, angle):
    try:
        board.digital[pin].write(angle)
    except:
        print("error")

board.digital[5].mode = SERVO #y
setServoAngle(5,90)

board.digital[9].mode = SERVO #X
setServoAngle(9,90)

#####PID#####
accError_x = 0
accError_y = 0
preError_x = 0
preError_y = 0
acc_delta_x = []
acc_delta_y = []
maxAngle = 13
step = 6
conver_rad = 70
def PIDcontrol(error, acc_error, pre_error, joint):
    if joint == 5:
        Kp = 26 #25
        Ki = 2 #0.1
        Kd = 6 #150 
    
    elif joint ==9:
        Kp = 23 #25
        Ki = 3  #0.1
        Kd = 7 #150 

    d_control = Kd*pre_error
    degree = Kp*error + Ki*acc_error + d_control

    if degree > maxAngle:
        degree = maxAngle
    elif degree< -maxAngle:
        degree = -maxAngle

    degree = degree + 90
    setServoAngle(joint,degree)

#######讀取影像########
cap = cv2.VideoCapture(1)

cv_file = cv2.FileStorage("data.xml", cv2.FILE_STORAGE_READ)
intrinsic = cv_file.getNode("intrinsic").mat()
distortion = cv_file.getNode("distortion").mat()
cv_file.release()
ori = [372,350]
orient = [0,0]
cnt = 0
steadyCnt = 0
print("program starting!")


r = 130
angle = 0 

while True:
    angle += step
    angle = angle%360
    orient[0] = ori[0]+r*math.cos(math.radians(angle))
    orient[1] = ori[1]+r*math.sin(math.radians(angle))

    start = time.time()
    position = []
    ret, frame = cap.read()
    h,  w = frame.shape[:2]
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(intrinsic, distortion, (w,h), 1, (w,h))
    dst = cv2.undistort(gray, intrinsic, distortion, None, newcameramtx)
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    dst = dst[4:692, 214:986]

    blurred = cv2.GaussianBlur(dst, (9,9),0)

    canny = cv2.Canny(blurred,30,150)

    _,contours, hierarchy = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for i in contours:
        if cv2.contourArea(i) > 5000 :
            x,y,w,h = cv2.boundingRect(i)
            #cv2.drawContours(dst, i, -1, 255,5)
            #test = str(int(x+w/2))+','+str(int(y+h/2))
            position.append(int(x+w/2))
            position.append(int(y+h/2))
            #cv2.putText(dst,test,(x,y),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2, cv2.LINE_AA)
            break

    #######移動球球#####
    if len(position) != 0 :
        stop = time.time()
        dur_time = stop-start

        cnt = 0
        delta_x =  (position[0] - orient[0])/349
        delta_y =  (position[1] - orient[1])/349

        acc_delta_x.append(delta_x*dur_time)
        acc_delta_y.append(delta_y*dur_time)
        
        while len(acc_delta_x)>20:
            acc_delta_x.pop(0)
            acc_delta_y.pop(0)
        
        accError_x = np.sum(accError_x)
        accError_y = np.sum(accError_y)

        PIDcontrol(delta_x, accError_x, (delta_x - preError_x)/dur_time, 9)
        PIDcontrol(delta_y, accError_y, (delta_y - preError_y)/dur_time, 5)

        preError_x = delta_x
        preError_y = delta_y

        if abs(delta_x*349) < conver_rad and abs(delta_y*349) <conver_rad:
            steadyCnt +=1
        else:
            steadyCnt -=15

        if steadyCnt >24:
            print("steady")
           
    else:
        cnt+=1
        if cnt > 24:

            preError_x = 0
            preError_y = 0

            acc_delta_x.clear()
            acc_delta_y.clear()

            setServoAngle(5,90)
            setServoAngle(9,89)
    dst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)        
    dst = cv2.circle(dst,(int(orient[0]),int(orient[1])),2,(0,0,255),-1)
    cv2.imshow("frame", dst)
    if (cv2.waitKey(33)&0xFF == ord('q')):
        break
cap.release()
cv2.destroyAllWindows()
cv2.waitKey(1)