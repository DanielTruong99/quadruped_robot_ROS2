#! /usr/bin/python3
# -*- coding: utf-8 -*-

# GUI
import sys
import numpy as np

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *
from controller_window import Ui_Form
from PyQt5 import QtCore, QtGui, QtWidgets

# ROS
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import Imu, Joy
#from std_msgs.msg import String

class GraphicsScene(QGraphicsScene):

    pressed = False

    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, 0, 0, 150, 150, parent = None) 
        self.opt = ""

    def setOption(self, opt):
        self.opt = opt

    def mouseReleaseEvent(self,event):
        self.pressed = False
        window.repaint(75, 75)
        window.cmd_Joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def mousePressEvent(self,event):
        self.pressed = True

    def mouseMoveEvent(self,event):
        if(self.pressed == True):
            x = event.scenePos().x()
            y = event.scenePos().y()
            if(((x - 75)**2 + (y - 75)**2) < 60**2):
                if((x-75) != 0):
                    theta = math.atan2((75-y),(x-75))
                else:
                    theta = 0

                if(window.ui.crab_radioButton.isChecked()):
                    window.cmd_Joy.axes = [0.0, 0.0, 0.0, -(math.sqrt((x - 75)**2 + (y - 75)**2)/60)*math.cos(theta),
                                           (math.sqrt((x - 75)**2 + (y - 75)**2)/60)*math.sin(theta), 0.0, 0.0, 0.0]
                else:
                    window.cmd_Joy.axes = [-(math.sqrt((x - 75)**2 + (y - 75)**2)/60)*math.cos(theta) , 0.0, 0.0,
                                            0.0, (math.sqrt((x - 75)**2 + (y - 75)**2)/60)*math.sin(theta), 0.0, 0.0, 0.0]
                window.repaint(x, y)
            #print("x : " + str(x) + ", y : " + str(y))


class GUI(QDialog):

    first_time = True
    enable_stand_up = False
    stand = False
    crawl = False
    trot = False
    normal_mode = False
    crab_mode = False
    IMU = False
    cnt = 0
    cnt_imu = 0

    def __init__(self,parent=None):
        # GUI
        super(GUI, self).__init__(parent)
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(10)

        self.repaint(75, 75)

        #ROS
        self.cmd_Joy = Joy()
        self.cmd_Joy.axes = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.cmd_Joy.buttons = [0,0,0,0,0,0,0,0,0,0,0]
        rclpy.init(args=None)
        self.pub_node = Node('pub_path')
        self.pub_Joy = self.pub_node.create_publisher(Joy, '/joy', 10)
 

    def repaint(self, x, y):
        if(self.first_time):
            self.scene = GraphicsScene()
            self.first_time = False
        self.ui.graphicsView_2.setScene(self.scene)
        self.scene.addEllipse(0, 0, 150, 150, QPen(QColor(0,182,110)), QBrush(QColor(0,182,110)))
        self.scene.addEllipse(x - 15, y - 15, 30, 30, QPen(Qt.red), QBrush(Qt.red))

    def update(self):

        if(self.stand):
            self.cmd_Joy.buttons[0] = 1
            self.cmd_Joy.buttons[1] = 0
            self.cmd_Joy.buttons[2] = 0
            self.cnt += 1
        elif(self.trot):
            self.cmd_Joy.buttons[0] = 0
            self.cmd_Joy.buttons[1] = 1
            self.cmd_Joy.buttons[2] = 0
            self.cnt += 1
        elif(self.crawl):
            self.cmd_Joy.buttons[0] = 0
            self.cmd_Joy.buttons[1] = 0
            self.cmd_Joy.buttons[2] = 1
            self.cnt += 1
        elif(self.enable_stand_up):
            self.cmd_Joy.buttons[6] = 1
            self.cnt += 1
        else:
            self.cmd_Joy.buttons[0] = 0
            self.cmd_Joy.buttons[1] = 0
            self.cmd_Joy.buttons[2] = 0
            self.cmd_Joy.buttons[6] = 0

        #if(self.IMU):
        #    self.cmd_Joy.buttons[7] = 1
        #else:
        #    self.cmd_Joy.buttons[7] = 0

        if(self.cnt >= 50):
            self.enable_stand_up = False
            self.stand = False
            self.crawl = False
            self.trot = False
            self.cnt = 0

        #print(self.cmd_Joy.buttons)
        self.pub_Joy.publish(self.cmd_Joy)

    def stand_up(self):
        print("stand up!")
        self.enable_stand_up = True
            
    def stand(self):
        print("stand")
        self.stand = True

    def crawl(self):
        print("crawl")
        self.crawl = True

    def trot(self):
        print("trot")
        self.trot = True

    def IMU(self):
        self.IMU = not self.IMU
        print(f"IMU:{self.IMU}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = GUI()
    window.show()
    sys.exit(app.exec_())
