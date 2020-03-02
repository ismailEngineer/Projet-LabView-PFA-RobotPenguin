import qrtools
import pyqrcode
import cv2 
import numpy as np 

#Create QrCode :
TheQrImage = pyqrcode.create("Article 1 Rang A Secteur D")
TheQrImage.png('QrcodeGenerated.png',scale=6)
TheQrImage.show() # show QrCode

"""
# Read QrCode :
qr = qrtools.QR()
print (qr.decode('QrcodeGenerated.png'))
dataQr = qr.data
print (dataQr)

"""
