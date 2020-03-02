import cv2

cap = cv2.VideoCapture(1)


while (cap.isOpened()):
	ret,frame = cap.read()
	if ret == True :
		cv2.imshow('Video',frame)

	if cv2.waitKey(1) & 0xFF == 27 :
		break 

cap.release()
cv2.destroyAllWindows()