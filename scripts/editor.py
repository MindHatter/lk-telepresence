# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'editor.ui'
#
# Created by: PyQt5 UI code generator 5.10
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Editor(object):
    def setupUi(self, Editor):
        Editor.setObjectName("Editor")
        Editor.resize(512, 669)
        self.ApplyButton = QtWidgets.QPushButton(Editor)
        self.ApplyButton.setGeometry(QtCore.QRect(10, 610, 231, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.ApplyButton.setFont(font)
        self.ApplyButton.setFocusPolicy(QtCore.Qt.NoFocus)
        self.ApplyButton.setObjectName("ApplyButton")
        self.RiseButton = QtWidgets.QPushButton(Editor)
        self.RiseButton.setGeometry(QtCore.QRect(270, 610, 231, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.RiseButton.setFont(font)
        self.RiseButton.setFocusPolicy(QtCore.Qt.NoFocus)
        self.RiseButton.setObjectName("RiseButton")
        self.TextEdit = QtWidgets.QPlainTextEdit(Editor)
        self.TextEdit.setGeometry(QtCore.QRect(10, 10, 491, 591))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.TextEdit.setFont(font)
        self.TextEdit.setObjectName("TextEdit")

        self.retranslateUi(Editor)
        QtCore.QMetaObject.connectSlotsByName(Editor)

    def retranslateUi(self, Editor):
        _translate = QtCore.QCoreApplication.translate
        Editor.setWindowTitle(_translate("Editor", "YAML Editor"))
        self.ApplyButton.setText(_translate("Editor", "Применить"))
        self.RiseButton.setText(_translate("Editor", "Восстановить"))

