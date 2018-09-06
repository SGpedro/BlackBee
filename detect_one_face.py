'''
Code used by the Black Bee Drones team.
(This code was not developed by us and it can be easily found on google.)
'''

#!/usr/bin/env python
# license removed for brevity
# importing the required libraries
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
#import do ROS
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image


def main():
    reconhecimento()


def reconhecimento():

    pub_face = rospy.Publisher("movimento_face", String, queue_size = 1)
    sub_camera = rospy.Subscriber("bebop/image_raw", Image,queue_size = 1, buff_size = 2**24)
    rospy.init_node("node_facedetection", anonymous=True)

    # loading the haard classifier (to recognize faces).
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    # some people can just use the haarcascade_frontalface_default.xml without specifying the path
    #test = face_cascade.load('../../opencv/data/haarcascades/haarcascade_frontalface_default.xml')
    test = face_cascade.load('../../../../opencv/data/haarcascades/haarcascade_frontalface_default.xml')

    # start the video capture
    video_capture = cv2.VideoCapture(0)

    # while-loop to detect face on webcam until you press 'q'.
    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = video_capture.read()
        frame = imutils.resize(frame, width=600)
        #convert the frame (of the webcam) to gray)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mask = cv2.erode(gray, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
            # detecting the faces

        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)

        bigest_area = 10
        interacao = 0
        # Draw a rectangle around the faces
        for (x, y, w, h) in faces:
            face_area = w * h

            if(face_area > bigest_area):
                bigest_area = face_area
                print(bigest_area)

                interacao += 1
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

                x_imagem = x + (w/2)
                y_imagem = y + (h/2)
                cv2.circle(frame, (x+(w/2), y+(h/2)), 5, (0,0,255), -1)

                if(x_imagem > 200 and x_imagem < 400 and interacao == 5):
                    interacao = 0
                    rospy.loginfo("CENTRO")
                elif(x_imagem < 200): #ROSTO PRA ESQUERDA, ENTAO VAI PARA DIREITA
                    rospy.loginfo("ROSTO NA ESQUERDA")
                    pub_face.publish("esq")
                elif(x_imagem > 400): #ROSTO PRA DIREITA, ENTAO VAI PARA ESQUERDA
                    rospy.loginfo("ROSTO NA DIREITA")
                    pub_face.publish("dir")

                cv2.imshow('Video', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("FIM DO PROGRAMA DE DETECCAO DE ROSTOS")
                video_capture.release()
                cv2.destroyAllWindows()
                break

if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass
