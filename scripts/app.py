#!/usr/bin/python
# -*- coding: utf-8 -*-

# Python Standart Library
import os
import cv2
import yaml
import datetime
import sys
# reload(sys)
# sys.setdefaultencoding('utf-8')

# PyQt
from PyQt5 import QtCore, QtGui, QtWidgets
import telegui
import editgui

# ROS
#import rosmaster
import roslaunch
import rospkg
import rospy
from std_msgs.msg import UInt8, String, Bool, Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Editor(QtWidgets.QDialog, editgui.Ui_Editgui):
    def __init__(self):
        # GUI init
        QtWidgets.QDialog.__init__(self)
        self.setupUi(self)

        rospack = rospkg.RosPack()
        self.pkg_path=rospack.get_path('telepresence')

        self.ApplyButton.clicked.connect(self.apply)
        self.RiseButton.clicked.connect(self.rise)

        self.cfg_path = os.path.join(self.pkg_path, 'config.yaml')
        with open(self.cfg_path, 'r') as cfg:
            data = cfg.read()
            self.load(data)
            self.YamlEdit.clear()
            self.YamlEdit.appendPlainText(data)

    def apply(self):
        os.system("cp {0} {0}.bak".format(self.cfg_path))
        with open(self.cfg_path, 'w') as cfg:
            cfg.write(self.YamlEdit.toPlainText())

    def rise(self):
        with open("{}.bak".format(self.cfg_path), 'r') as cfg:
            self.YamlEdit.clear()
            self.YamlEdit.appendPlainText(cfg.read())

    def load(self, data):
        data = yaml.load(data)
        for key in data.keys():
            print(key, str(data[key]))
            os.environ[key] = str(data[key])
        print('Load params')


class Telepresence(QtWidgets.QDialog, telegui.Ui_Telegui):
    def __init__(self):
        # GUI init
        QtWidgets.QDialog.__init__(self)
        self.setupUi(self)

        # GUI elems handle
        self.MicSlider.valueChanged.connect(self.mic_change)
        self.HeadsetSlider.valueChanged.connect(self.hs_change)
        self.MicCheck.toggled.connect(self.mic_mute)
        self.HeadsetCheck.toggled.connect(self.hs_mute)
        self.UpButton.clicked.connect(self.move_up)
        self.DownButton.clicked.connect(self.move_down)
        self.LeftButton.clicked.connect(self.move_left)
        self.RightButton.clicked.connect(self.move_right)
        self.StopButton.clicked.connect(self.move_stop)
        self.LaunchButton.clicked.connect(self.launch)
        self.ConfigButton.clicked.connect(self.config)
        self.RecoOnCheck.toggled.connect(self.reco_on)

        # ROS Init
        rospy.init_node('telegui')
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('telepresence')
        self.log_path = os.path.join(self.pkg_path, 'logs')

        rospy.Subscriber("/cv_camera/image_raw", Image, self.video_cap)
        rospy.Subscriber("/battery", UInt8, self.battery)
        rospy.Subscriber("/recognized", String, self.logout)
        self.teleop_pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
        self.reco_on_pub = rospy.Publisher("/reco_on", Bool, queue_size=1)
        self.reco_toggle_pub = rospy.Publisher("/reco_toggle", Empty, queue_size=1)
        self.is_reco = False
        self.is_launch = False
        self.twist = Twist() 
        self.bridge = CvBridge()

        self.move_values = {
            87: ('forward',(2.0, 0.0)),
            83: ('backward',(-2.0, 0.0)),
            65: ('left',(0.0, 2.0)),
            68: ('right',(0.0, -2.0)),
            32: ('stop',(0.0, 0.0)),
        }

        self.log = ''

        os.system("amixer set Capture nocap")

    def reco_on(self):
        if self.is_launch:
            if self.RecoOnCheck.isChecked():
                self.is_reco = True
            else:
                self.is_reco = False
            self.reco_on_pub.publish(self.is_reco)

    def dt_cut(self, dt):
        return str(dt).split('.')[0]

    def log_save(self):
        path = os.path.join(self.log_path, "{}.txt".format(self.dt_cut(datetime.datetime.now())))
        print (path)
        with open(path, 'w') as f:
            f.write(self.log)

    def logout(self, msg):
        try:
            msg = msg.data
        except:
            pass

        msg = '{} {}'.format(self.dt_cut(datetime.datetime.now()), msg)
        self.LogBrowser.append(msg)
        self.log += '{}\n'.format(msg)

    def battery(self, msg):
        self.BatteryBar.setValue(msg.data)

    def video_cap(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            image = QtGui.QImage(cv_image, cv_image.shape[1], cv_image.shape[0], cv_image.shape[1] * cv_image.shape[2], QtGui.QImage.Format_RGB888)
            pixmap = QtGui.QPixmap()
            pixmap.convertFromImage(image.rgbSwapped())
            pixmap = pixmap.scaled(800, 600, QtCore.Qt.KeepAspectRatio)
            self.Video.setPixmap(pixmap)
        except CvBridgeError as e:
            pass

    def move_gen_and_pub(self, key):
        if self.is_launch:
            move_value = self.move_values[key]
            self.twist.linear.x = move_value[-1][0]
            self.twist.angular.z = move_value[-1][1]
            self.teleop_pub.publish(self.twist)
            self.logout("Move command: {}".format(move_value[0]))

    def keyPressEvent(self, event):
        key = event.key()
        print(key)
        if key in self.move_values.keys():
            self.move_gen_and_pub(key)
        if key == 16777248:
            os.system("amixer set Capture cap")
            self.logout("Mic is unmuted")
            if self.is_reco:
                self.reco_toggle_pub.publish()

    def keyReleaseEvent(self, event):
        key = event.key()
        #print(key)
        if key == 16777248:
            os.system("amixer set Capture nocap")
            self.logout("Mic is muted")
            if self.is_reco:
                self.reco_toggle_pub.publish()

    def move_up(self):
        self.move_gen_and_pub(87)
    def move_down(self):
        self.move_gen_and_pub(83)
    def move_left(self):
        self.move_gen_and_pub(65)
    def move_right(self):
        self.move_gen_and_pub(68)
    def move_stop(self):
        self.move_gen_and_pub(32)

    def launch(self):
        if self.is_launch:
            self.LaunchButton.setEnabled(False)
            # self.remote_launch.shutdown()
            # self.local_launch.shutdown()
            self.tele_launch.shutdown()
            self.Video.clear()
            self.Video.setText('Видео отсутствует')
            self.LaunchButton.setText('Запустить')
            self.LaunchButton.setEnabled(True)
            self.log_save()
            self.log = ''
        else:
            self.LaunchButton.setEnabled(False)
            try:
                self.tele_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.pkg_path+"/launch/telepresence.launch"])
                self.tele_launch.start()
                # self.remote_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.pkg_path+"/launch/remote.launch"])
                # self.local_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.pkg_path+"/launch/local.launch"])
                # self.remote_launch.start()
                # self.local_launch.start()
                self.LaunchButton.setText('Остановить')
            except roslaunch.core.RLException as e:
                self.logout("Error: {}".format(str(e)))
                self.is_launch = not self.is_launch
            self.LaunchButton.setEnabled(True)
        self.is_launch = not self.is_launch

    def mic_change(self):
        value = self.MicSlider.value()
        os.system("amixer set Capture {}%".format(value))
        self.logout("Mic value: {}".format(value))
        os.system("amixer set Capture nocap")

    def hs_change(self):
        value = self.HeadsetSlider.value()
        os.system("amixer set Master {}%".format(value))
        self.logout("Audio value: {}".format(value))

    def mic_mute(self):
        if self.MicCheck.isChecked():
            os.system("amixer set Capture nocap")
            self.MicSlider.setEnabled(False)
            self.logout("Mic is muted")
        else:
            os.system("amixer set Capture cap")
            self.MicSlider.setEnabled(True)
            self.logout("Mic is unmuted")

    def hs_mute(self):
        if self.HeadsetCheck.isChecked():
            os.system("amixer -D pulse set Master mute")
            self.HeadsetSlider.setEnabled(False)
            self.logout("Audio is muted")

        else:
            os.system("amixer -D pulse set Master unmute")
            self.HeadsetSlider.setEnabled(True)
            self.logout("Audio is unmuted")

    def config(self):
        edit.show()


if __name__ == "__main__":
    # master = rosmaster.master.Master()
    # master.start()
    app = QtWidgets.QApplication(sys.argv)
    tele = Telepresence()
    tele.show()
    edit = Editor()
    app.exec_()
    # master.stop()
