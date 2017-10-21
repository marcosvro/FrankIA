import numpy as np
import cv2
from matplotlib import pyplot as plt
import socket

host = '127.0.0.1'     # Endereco IP do Servidor
port = 666             # Porta que o Servidor esta
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
dest = (host, port)

cap = cv2.VideoCapture(1)

while cap.isOpened():
	# Capture frame-by-frame
	ret, img = cap.read()
	if not ret:
		continue
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

	# noise removal
	kernel = np.ones((3,3),np.uint8)
	opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)

	# sure background area
	sure_bg = cv2.dilate(opening,kernel,iterations=3)
	#cv2.imshow('path',sure_bg)
	cv2.imshow('original', img)

	#process
	width = sure_bg.shape[1]
	height = sure_bg.shape[0]
	center_x = int(width/2)
	center_y = int(height/2)
	#print (sure_bg[center_y, center_x])
	
	indice_better = 0
	dist_better = 0
	for i in range(center_x, width, 1):
		if sure_bg[height-1, i]:
			break

		try:
			dist_aux = height-1 - sure_bg[:height, i].nonzero()[0][-1]
		except IndexError:
			break

		if dist_aux > dist_better:
			dist_better = dist_aux
			indice_better = i

	for i in range(center_x, 0, -1):
		if sure_bg[height-1, i]:
			break

		try:
			dist_aux = height-1 - sure_bg[:height, i].nonzero()[0][-1]
		except IndexError:
			break

		if dist_aux > dist_better:
			dist_better = dist_aux
			indice_better = i
	result = cv2.cvtColor(sure_bg,cv2.COLOR_GRAY2BGR)
	pad_vertical = 5
	
		
	if dist_better:
		print ("indice: ", indice_better)
		udp.sendto (str(indice_better).encode('utf-8'), dest)
		if indice_better > center_x:
			result[center_y-pad_vertical:center_y+pad_vertical, center_x:indice_better] = (0, 0, 255)
		else:
			result[center_y-pad_vertical:center_y+pad_vertical, indice_better:center_x] = (0, 0, 255)
	else:
		print ("Cego!!")
	
	cv2.imshow('result',result)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
	#print ("OK!!")

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
udp.close()


