   
import numpy as numpy
import cv2
import cv2.aruco as aruco
import math
import datetime
import time
import glob
from socket import *
def make_1080p():
    cap.set(3, 1920)
    cap.set(4, 1080)
    #i = 0
    #x = (0, 0)
    #y = (0, 0)
    #theta = 0
    #j=0
    #g=0
    #phi = 0
class movement:    
    def __init__(self,i,x,y,theta,j,g,phi,bot,goal):
        self.i=i
        self.x=x
        self.y=y
        self.theta=theta
        self.j=j
        self.g=g
        self.phi=phi
        self.bot=bot
        self.goal=goal


    def distance(self,pt1, pt2):
        x = pt1[0] - pt2[0]
        y = pt1[1] - pt2[1]
        distance = math.sqrt(x*x + y*y)
        #print("distance: ",distance)
        return distance

    def angle_calculate(self,pt1, pt2):

        a = pt2[0]-pt1[0]
        b = pt2[1]-pt1[1]
        angle = math.degrees(math.atan2(b, a))
        if(angle<0):
              angle = 360 + angle
        return int(angle)

    def allignment(self,theta, phi,dist,ip):
        global g
        clientSocket1 = socket(AF_INET, SOCK_DGRAM)
        clientSocket1.settimeout(1)
        print("angle difference: ",(theta - phi))
        addr1 = (ip, 5007)
        if -10<=theta-phi<=10:
            if(self.g<=3):       
                clientSocket1.sendto('4'.encode(), addr1)
                self.g=self.g+1
                print(4)
            else:
                #g=0
                if (dist>140):
                    clientSocket1.sendto('0'.encode(), addr1)
                    print(0)
                else:
                    clientSocket1.sendto('4'.encode(), addr1)
                    print(4)
                
        else:
            if  ((0<= theta-phi <=180) | (180 <=phi-theta <= 360)):
                clientSocket1.sendto('3'.encode(), addr1)
                print(3)
            #elif 180<= theta-phi <=360 | -180<= theta-phi <=0:
            else:
                clientSocket1.sendto('2'.encode(), addr1)
                print(2) 

    def aruco_detect(self,frame, robot,ip):
        #global i,j
        #i = 0
        #global x 
        #x = (0, 0)
        #global y 
        #y = (0, 0)
        #global theta
        #theta = 0
        #global phi
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()

        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        aruco_frame = aruco.drawDetectedMarkers(gray, corners)
        #print("corners")
        #print(len(corners))
        if len(corners)>0:
            for marker in range(len(ids)):

                #print(len(corners))
                #print(ids)
                x_center= int((corners[marker][0][0][0] + corners[marker][0][1][0] + corners[marker][0][2][0] + corners[marker][0][3][0])/4)
                y_center= int((corners[marker][0][0][1] + corners[marker][0][1][1] + corners[marker][0][2][1] + corners[marker][0][3][1])/4)

                #print(x_center, y_center)
                

                cv2.circle(frame, (x_center, y_center),2,(0,0,255),2)#red dot at center
                x1 = int(corners[marker][0][0][0])
                x3 = int(corners[marker][0][3][0])
                y1 = int(corners[marker][0][0][1])
                y3 = int(corners[marker][0][3][1])

                pt1 = (x3,y3)
                pt2 = (x1,y1)
                #   Head
                #    _____
                # pt2|      |
                #    |      |
                #    |      |
                # pt1|      |
                #    --------
                cv2.circle(frame, pt1, 2, (0,0,255), 2)#corner
                cv2.circle(frame,pt2, 2, (0,0,255), 2)#corner
                cv2.imshow('aruco_frame', frame)
                if ids[marker] == self.bot:
                    pt1 = (x3,y3)
                    pt2 = (x1,y1)
                    self.theta = self.angle_calculate(pt1, pt2)
                    print('theta', self.theta)
                    self.x = (x_center, y_center)
                
                elif ids[marker] == self.goal:
                    self.y = (x_center, y_center)

                cv2.imshow('aruco_frame', frame)
                robot[int(ids[marker])]=(int(x_center), int(y_center), int(self.theta))
                if not (self.j == 2) :
                       print("YES")
                       self.j+=1
                       self.phi = self.angle_calculate(self.x, self.y)
    	
            print("phi: ",self.phi)
            dist = self.distance(self.x,self.y)
            print("distance:",dist)
            if len(ids)>1:
                self.allignment(self.theta, self.phi,dist,ip)
                self.j=2
        start = 0
        start_time_update = time.time()

        cv2.imshow("aruco_frame", frame)
        return robot
robot={}
cap = cv2.VideoCapture(0)
make_1080p()
robot1=movement(0,(0,0),(0,0),0,0,0,0,1,2)
#robot2=movement(0,(0,0),(0,0),0,0,0,0,1,2)
#robot3=movement(0,(0,0),(0,0),0,0,0,0,6,5)
while(1):
     _,img_rgb = cap.read()
     robot = robot1.aruco_detect(img_rgb,robot,"192.168.0.129")#1
     #robot = robot2.aruco_detect(img_rgb,robot,"192.168.0.107")
     #robot = robot3.aruco_detect(img_rgb,robot,"192.168.0.103")
     k =  cv2.waitKey(1) & 0xFF
     if k == 27:
        cap.release()
        cv2.destroyAllWindow()
        break
