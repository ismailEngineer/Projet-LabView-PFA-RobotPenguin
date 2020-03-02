import qrcode
d = qrcode.Decoder()

if d.decode('Qrimage.png') :
	print(d.result)