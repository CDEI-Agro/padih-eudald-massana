# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'c:\Dades\Projectes\Actius\Rovinya\Comandament\comandamentView.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(932, 534)
        self.graphicsView = QtWidgets.QGraphicsView(Dialog)
        self.graphicsView.setGeometry(QtCore.QRect(0, 0, 841, 461))
        self.graphicsView.setObjectName("graphicsView")
        self.mesButton = QtWidgets.QPushButton(Dialog)
        self.mesButton.setGeometry(QtCore.QRect(850, 10, 75, 41))
        self.mesButton.setObjectName("mesButton")
        self.menysButton = QtWidgets.QPushButton(Dialog)
        self.menysButton.setGeometry(QtCore.QRect(850, 60, 75, 41))
        self.menysButton.setObjectName("menysButton")
        self.afegirButton = QtWidgets.QPushButton(Dialog)
        self.afegirButton.setGeometry(QtCore.QRect(10, 470, 75, 23))
        self.afegirButton.setObjectName("afegirButton")
        self.latEdit = QtWidgets.QLineEdit(Dialog)
        self.latEdit.setGeometry(QtCore.QRect(160, 470, 113, 20))
        self.latEdit.setText("")
        self.latEdit.setObjectName("latEdit")
        self.lonEdit = QtWidgets.QLineEdit(Dialog)
        self.lonEdit.setGeometry(QtCore.QRect(340, 470, 113, 20))
        self.lonEdit.setObjectName("lonEdit")
        self.obreButton = QtWidgets.QPushButton(Dialog)
        self.obreButton.setGeometry(QtCore.QRect(850, 440, 75, 23))
        self.obreButton.setObjectName("obreButton")
        self.label = QtWidgets.QLabel(Dialog)
        self.label.setGeometry(QtCore.QRect(110, 480, 47, 14))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(Dialog)
        self.label_2.setGeometry(QtCore.QRect(300, 480, 47, 14))
        self.label_2.setObjectName("label_2")
        self.netejaButton = QtWidgets.QPushButton(Dialog)
        self.netejaButton.setGeometry(QtCore.QRect(850, 110, 75, 41))
        self.netejaButton.setObjectName("netejaButton")
        self.connectaButton = QtWidgets.QPushButton(Dialog)
        self.connectaButton.setGeometry(QtCore.QRect(850, 470, 75, 51))
        self.connectaButton.setObjectName("connectaButton")
        self.cmdEdit = QtWidgets.QLineEdit(Dialog)
        self.cmdEdit.setGeometry(QtCore.QRect(560, 470, 191, 20))
        self.cmdEdit.setObjectName("cmdEdit")
        self.enviaButton = QtWidgets.QPushButton(Dialog)
        self.enviaButton.setGeometry(QtCore.QRect(760, 470, 75, 23))
        self.enviaButton.setObjectName("enviaButton")
        self.label_3 = QtWidgets.QLabel(Dialog)
        self.label_3.setGeometry(QtCore.QRect(500, 470, 47, 14))
        self.label_3.setObjectName("label_3")
        self.coorMsg = QtWidgets.QLabel(Dialog)
        self.coorMsg.setGeometry(QtCore.QRect(20, 490, 421, 20))
        self.coorMsg.setObjectName("coorMsg")
        self.errorMsg = QtWidgets.QLabel(Dialog)
        self.errorMsg.setGeometry(QtCore.QRect(450, 490, 71, 16))
        self.errorMsg.setObjectName("errorMsg")
        self.posicioCB = QtWidgets.QCheckBox(Dialog)
        self.posicioCB.setGeometry(QtCore.QRect(850, 310, 67, 18))
        self.posicioCB.setObjectName("posicioCB")
        self.obreMapaButton = QtWidgets.QPushButton(Dialog)
        self.obreMapaButton.setGeometry(QtCore.QRect(850, 400, 75, 23))
        self.obreMapaButton.setObjectName("obreMapaButton")
        self.obrePuntsButton = QtWidgets.QPushButton(Dialog)
        self.obrePuntsButton.setGeometry(QtCore.QRect(850, 360, 75, 23))
        self.obrePuntsButton.setObjectName("obrePuntsButton")
        self.label_4 = QtWidgets.QLabel(Dialog)
        self.label_4.setGeometry(QtCore.QRect(860, 170, 47, 14))
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.vbat = QtWidgets.QLabel(Dialog)
        self.vbat.setGeometry(QtCore.QRect(860, 190, 47, 14))
        self.vbat.setAlignment(QtCore.Qt.AlignCenter)
        self.vbat.setObjectName("vbat")
        self.joypos = QtWidgets.QLabel(Dialog)
        self.joypos.setGeometry(QtCore.QRect(560, 490, 191, 16))
        self.joypos.setObjectName("joypos")
        self.puls1 = QtWidgets.QLabel(Dialog)
        self.puls1.setGeometry(QtCore.QRect(50, 510, 71, 16))
        self.puls1.setObjectName("puls1")
        self.puls2 = QtWidgets.QLabel(Dialog)
        self.puls2.setGeometry(QtCore.QRect(170, 510, 71, 16))
        self.puls2.setObjectName("puls2")
        self.puls3 = QtWidgets.QLabel(Dialog)
        self.puls3.setGeometry(QtCore.QRect(290, 510, 61, 16))
        self.puls3.setObjectName("puls3")
        self.puls4 = QtWidgets.QLabel(Dialog)
        self.puls4.setGeometry(QtCore.QRect(430, 510, 81, 16))
        self.puls4.setObjectName("puls4")
        self.puls5 = QtWidgets.QLabel(Dialog)
        self.puls5.setGeometry(QtCore.QRect(590, 510, 81, 16))
        self.puls5.setObjectName("puls5")
        self.puls6 = QtWidgets.QLabel(Dialog)
        self.puls6.setGeometry(QtCore.QRect(740, 510, 81, 16))
        self.puls6.setObjectName("puls6")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Comandament"))
        self.mesButton.setText(_translate("Dialog", "Zoom +"))
        self.menysButton.setText(_translate("Dialog", "Zoom -"))
        self.afegirButton.setText(_translate("Dialog", "Afegir punt"))
        self.obreButton.setText(_translate("Dialog", "Obre Cami"))
        self.label.setText(_translate("Dialog", "Lat."))
        self.label_2.setText(_translate("Dialog", "Lon."))
        self.netejaButton.setText(_translate("Dialog", "Neteja"))
        self.connectaButton.setText(_translate("Dialog", "Connecta"))
        self.enviaButton.setText(_translate("Dialog", "Envia"))
        self.label_3.setText(_translate("Dialog", "Comanda"))
        self.coorMsg.setText(_translate("Dialog", "Coordenades"))
        self.errorMsg.setText(_translate("Dialog", "ERROR"))
        self.posicioCB.setText(_translate("Dialog", "Posició"))
        self.obreMapaButton.setText(_translate("Dialog", "Obre Mapa"))
        self.obrePuntsButton.setText(_translate("Dialog", "Obre Punts"))
        self.label_4.setText(_translate("Dialog", "V.Bat"))
        self.vbat.setText(_translate("Dialog", "0.0"))
        self.joypos.setText(_translate("Dialog", "joypos"))
        self.puls1.setText(_translate("Dialog", "Pulsador 1"))
        self.puls2.setText(_translate("Dialog", "Pulsador 2"))
        self.puls3.setText(_translate("Dialog", "Pulsador 3"))
        self.puls4.setText(_translate("Dialog", "Pulsador 4"))
        self.puls5.setText(_translate("Dialog", "Pulsador 5"))
        self.puls6.setText(_translate("Dialog", "Pulsador 6"))
