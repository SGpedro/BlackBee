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
from cv_bridge import CvBridge, CvBridgeError


#image transport package plugin

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.sub_camera = rospy.Subscriber("/bebop/image_raw", Image, self.callback, queue_size = 1, buff_size = 2**24)
        self.pub_face = rospy.Publisher("movimento_face", String, queue_size = 1)
        # loading the haard classifier (to recognize faces).
        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        self.test = self.face_cascade.load('../../../../opencv/data/haarcascades/haarcascade_frontalface_default.xml')

    def callback(self, img):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "mono8")
        except CvBridgeError as e:
            print(e)
        self.reconhecimento(cv_image)

    def reconhecimento(self, imagem_convertida):
        print("to no reconhecimento")

        gray = imagem_convertida

        gray = imutils.resize(gray, width=600)
        #convert the frame (of the webcam) to gray)
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mask = cv2.erode(gray, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # detecting the faces

        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)


        bigest_area = 10
        #interacao = 0;
        # Draw a rectangle around the faces
        for (x, y, w, h) in faces:
            face_area = w * h

            if(face_area > 4000):
                bigest_area = face_area
                print(bigest_area)

                #cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

                x_imagem = x + (w/2)
                y_imagem = y + (h/2)
                #cv2.circle(frame, (x+(w/2), y+(h/2)), 5, (0,0,255), -1)

                if(x_imagem > 200 and x_imagem < 400):
                    rospy.loginfo("CENTRO")
                    interacao = 0
                elif(x_imagem < 200): #ROSTO PRA ESQUERDA, ENTAO VAI PARA DIREITA
                    rospy.loginfo("ROSTO NA ESQUERDA")
                    interacao = 0
                    self.pub_face.publish("esq")
                elif(x_imagem > 400): #ROSTO PRA DIREITA, ENTAO VAI PARA ESQUERDA
                    rospy.loginfo("ROSTO NA DIREITA")
                    interacao = 0
                    self.pub_face.publish("dir")

                #cv2.imshow('Video', frame)
                cv2.waitKey(1)

                '''if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("FIM DO PROGRAMA DE DETECCAO DE ROSTOS")
                    video_capture.release()
                    cv2.destroyAllWindows()
                    break'''



def main():
    print("at main")
    rospy.init_node("node_facedetection", anonymous=True)
    instancia_classe = image_converter()
    rospy.spin()


if __name__ == '__main__':
    try:
        print("Goin' to main")
        main()
    except:
        rospy.ROSInterruptException
        pass
