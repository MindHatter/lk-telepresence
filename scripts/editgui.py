# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'editgui.ui'
#
# Created by: PyQt5 UI code generator 5.10
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Editgui(object):
    def setupUi(self, Editgui):
        Editgui.setObjectName("Editgui")
        Editgui.resize(512, 669)
        self.ApplyButton = QtWidgets.QPushButton(Editgui)
        self.ApplyButton.setGeometry(QtCore.QRect(10, 610, 231, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.ApplyButton.setFont(font)
        self.ApplyButton.setFocusPolicy(QtCore.Qt.NoFocus)
        self.ApplyButton.setObjectName("ApplyButton")
        self.RiseButton = QtWidgets.QPushButton(Editgui)
        self.RiseButton.setGeometry(QtCore.QRect(270, 610, 231, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.RiseButton.setFont(font)
        self.RiseButton.setFocusPolicy(QtCore.Qt.NoFocus)
        self.RiseButton.setObjectName("RiseButton")
        self.YamlEdit = QtWidgets.QPlainTextEdit(Editgui)
        self.YamlEdit.setGeometry(QtCore.QRect(10, 10, 491, 591))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.YamlEdit.setFont(font)
        self.YamlEdit.setObjectName("YamlEdit")

        self.retranslateUi(Editgui)
        QtCore.QMetaObject.connectSlotsByName(Editgui)

    def retranslateUi(self, Editgui):
        _translate = QtCore.QCoreApplication.translate
        Editgui.setWindowTitle(_translate("Editgui", "YAML Editor"))
        self.ApplyButton.setText(_translate("Editgui", "Применить"))
        self.RiseButton.setText(_translate("Editgui", "Восстановить"))

