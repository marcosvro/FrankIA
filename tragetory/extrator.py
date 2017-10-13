import numpy as np
import threading
#import cv2
import time
import math
import os
import spidev
import ikpy as ik
import serial


#CONFIGS +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
deslocamentoZpes = 2.
deslocamentoXpes = 7.
deslocamentoYpelves = 4.
periodo = 0.25
nEstados = 200
dMovx = deslocamentoXpes/nEstados
frameRate = periodo/nEstados
data_foot = np.zeros((nEstados,8), dtype=np.uint8)
data_pelv = np.zeros((nEstados,8), dtype=np.uint8)


#perna - quadril = target
link0 = ik.link.URDFLink("calc_lateral", [0,0,1.9], [0,0,0], [1,0,0], use_symbolic_matrix=True, bounds=(-30,30))
link1 = ik.link.URDFLink("calc_frontal", [0,0,0.7], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds=(-60,60))
link2 = ik.link.URDFLink("joelho", [0,0,8.2] , [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds=(-60,60))
link3 = ik.link.URDFLink("quadril", [0,0,6.4], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds=(-80,80))
link4 = ik.link.URDFLink("pelves", [0, 1.7, 4], [0, 0, 0], [1, 0, 0], use_symbolic_matrix=True, bounds=(-50,50))

#perna - pe = target
link5 = ik.link.URDFLink("pelves", [0,0,0], [0,0,0], [1,0,0], use_symbolic_matrix=True, bounds = (-50, 50))
link6 = ik.link.URDFLink("quadril", [0,-1.7,-4], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds = (-80, 80))
link7 = ik.link.URDFLink("joelho", [0,0,-6.4], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds = (-60, 60))
link8 = ik.link.URDFLink("calc_frontal", [0,0,-8.2], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds = (-60, 60))
link9 = ik.link.URDFLink("calc_lateral", [0,0,-0.7], [0,0,0], [1,0,0], use_symbolic_matrix=True, bounds = (-30, 30))
link10 = ik.link.URDFLink("pe", [0,0,-0.7], [0,0,0], [0,0,0], use_symbolic_matrix=True)


#Parametros de DH
L = np.array([4.5, 8.3, 6.45, 8.24, 0.], dtype=float)
frank_dir = [[np.pi/2., 0., -np.pi/2., 0., 0., 0., 0.],[-L[1], 0., 0., 0., 0., 0., 0.],[-L[0], 0., 0., L[2], L[3], 0., L[4]],[0., np.pi/2., -np.pi/2., 0., 0., np.pi/2., 0.]]
frank_esq = [[np.pi/2., 0., -np.pi/2., 0., 0., 0., 0.],[-L[1], 0., 0., 0., 0., 0., 0.],[L[0], 0., 0., L[2], L[3], 0., L[4]],[0., np.pi/2., -np.pi/2., 0., 0., np.pi/2., 0.]]
mat1 = np.array(frank_dir, dtype=float)
mat2 = np.array(frank_esq, dtype=float)

#AB_6
ab6 = np.array([[0.,0.,1.,0.],[0.,1.,0.,-4.5],[-1.,0.,0.,-22.99],[0.,0.,0.,1.]], dtype=float)



#chains
foot2pelv = ik.chain.Chain([link0, link1, link2, link3, link4], [True, True, True, False, False])
pelv2foot = ik.chain.Chain([link5, link6, link7, link8, link9, link10], [True, True, True, True, False, False])


#start joint positions
jointsf2p = np.deg2rad([0, 13.24660153,-30.31297739,17.0563224,0])
jointsp2f = np.deg2rad([0,-17.0563224,30.31297739,-13.24660153,0,0])


#start target position
pos_inicial_pelves = [0., 0., 14.]
pos_inicial_pe = [0., 0., 14.]


#COMUNICACAO +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
ser = serial.Serial('/dev/ttyUSB0', 115200)
#ser_uno = serial.Serial('/dev/ttyACM0', 115200)

#FUNCOES +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def fowa_cinematic(transform, MDH,N):
	xi_1 = transform[:3, 0]
	yi_1 = transform[:3, 1]
	zi_1 = transform[:3, 2]
	pk_1 = transform[:3, 3]
	for i in range(N):
		o = MDH[0, i]
		d = MDH[0, i]
		a = MDH[0, i]
		s = MDH[0, i]
		
		xi = xi_1*math.cos(o) + yi_1*math.sin(o)
		aux = np.cross(xi, zi_1)
		zi = zi_1*math.cos(s) + aux*math.sin(s)
		yi = np.cross(zi, xi)

		pk = d*zi_1 + a*xi + pk_1

		xi_1 = xi
		yi_1 = yi
		zi_1 = zi
		pk_1 = pk
	return [xi_1, yi_1, zi_1, pk_1]

def back_cinematic(transform, MDH,N):
	xi = transform[:3, 0]
	yi = transform[:3, 1]
	zi = transform[:3, 2]
	pk = transform[:3, 3]
	for i in range(6, N, -1):
		o = MDH[0, i]
		d = MDH[1, i]
		a = MDH[2, i]
		s = MDH[3, i]
		
		zi_1 = zi*math.cos(s) + yi*math.sin(s)
		aux = np.cross(xi, zi_1)
		xi_1 = xi*math.cos(o) + aux*math.sin(o)
		yi_1 = np.cross(zi_1, xi_1)

		pk_1 = -d*zi_1 - a*xi + pk

		xi = xi_1
		yi = yi_1
		zi = zi_1
		pk = pk_1
	return [xi, yi, zi, pk]

def invKinematic(A60):
	#falta calcular o sinal dos angulos
	#definir as constantes
	l1 = L[0]
	l2 = L[1]
	l3 = L[2]
	l4 = L[3]
	l5 = L[4]
	nx = A60[0]
	ny = A60[1]
	nz = A60[2]
	sx = A60[0]
	sy = A60[1]
	sz = A60[2]
	ax = A60[0]
	ay = A60[1]
	az = A60[2]
	px = A60[0]
	py = A60[1,3]
	pz = A60[2,3]

	#calculo de o4
	x4 =((px + l5)*(px + l5) + py *py + pz*pz -l3*l3 -l4*l4 )/(2*l3*l4)
	y4 = math.sqrt(1-x4*x4)
	o4 = math.atan2(y4,x4)
	o4 = math.round(o4,8)
	s4 = math.sin(o4)
	c4 = math.cos(o4)
	#calculo de o5
	xa = c4*l3 + l4
	ya = s4*l3
	oa = math.atan2(ya,xa)
	oa = math.round(oa,8)

	y5 = -pz
	x5 = math.sqrt((px + l5)*(px + l5) + py*py)
	o5 = math.atan2(y5,x5) - oa
	o5 = math.round(o5,8)
	c5 = math.cos(o5)
	#calculo de o6
	y6 = py
	x6 = -px - l5
	o6 = math.atan2(y6,x6)
	aux = math.cos(o4+o5)*l3 + c5*l4
	if(aux < 0):
		o6 = o6 + ny.pi
	s6 = math.sin(o6)
	c6 = math.cos(o6)
	#calculo de o2
	x2 = s6*ax + c6*ay
	y2 = math.sqrt(1 -x2*x2)
	o2 = math.atan2(y2,x2)
	s2 = math.sin(o2)
	#calculo de o1
	x1 = -s6*nx - c6*ny
	y1 = -s6*sx - c6*sy
	o1 = math.atan2(y1,x1)
	o1 = math.round(o1,8)
	if(s2 < 0):
		o1 = o1 + ny.pi
	#calculo de o3
	x345 = az
	y345 = c6*ax - s6*ay
	o345 = math.atan2(y345,x456)
	o345 = math.round(o345,8)
	if(s2 < 0):
		o345 = o345 + ny.pi
	o3 = o345 - o4 - o5

	return [o1,o2,o3,o4,o5,o6]

def Quaternion_toEulerianAngle(x, y, z, w):
	ysqr = y*y

	t0 = +2.0 * (w * x + y*z)
	t1 = +1.0 - 2.0 * (x*x + ysqr)
	X = math.degrees(math.atan2(t0, t1))

	t2 = +2.0 * (w*y - z*x)
	t2 =  1 if t2 > 1 else t2
	t2 = -1 if t2 < -1 else t2
	Y = math.degrees(math.asin(t2))

	t3 = +2.0 * (w * z + x*y)
	t4 = +1.0 - 2.0 * (ysqr + z*z)
	Z = math.degrees(math.atan2(t3, t4))

	return X, Y, Z


def mapeia(x, y, z):
	a = (x*y)/z
	return a

#SETUP +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
iner = np.array([0., 0., 0.], dtype=np.uint8)

#reading file
try:
	with open('data_foot.txt', 'r') as f:
		data_foot = np.loadtxt('data_foot.txt').reshape((nEstados,8))
		print ("File data_foot loaded!")
	with open('data_pelv.txt', 'r') as f:
		data_pelv = np.loadtxt('data_pelv.txt').reshape((nEstados,8))
		print ("File data_pelv loaded!")
except IOError:
	print "Error"
	exit()



#LOOP +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
start = time.time()
t = 0.
t_fps = 0.
t_state = 0.
t_inercial = 0.
state = 0
fps = 0
perna = 1
incli = [128., 128.]

for i in range(nEstados):
	#data_pelv[i][0] = (data_pelv[i][0]-90)*-1 + 90
	data_foot[i][0] = (data_foot[i][0]-90)*-1 + 90
	data_pelv[i][2] = (data_pelv[i][2]-90)*-1 + 90
	data_foot[i][2] = (data_foot[i][2]-90)*-1 + 90
	data_pelv[i][3] = data_pelv[i][3] - 5
	data_foot[i][3] = data_foot[i][3] - 5
	#data_pelv[i][1] = 128
	#data_pelv[i][4] = 90
	data_foot[i][4] = 90


while 1:
	break
	dtime = time.time() - start
	start = time.time()
	t_fps += dtime

	if(t_fps >= 1.):
		os.system("clear")
		print "fps: ", fps
		t_fps = 0.
		fps = 0
	fps = fps + 1
	#fowa_cinematic(np.eye(4), mat1,6)
	T = back_cinematic(np.eye(4),mat2,0)
	invKinematic(T)
	

while 1:
	#sending data
	########################################	incluir iner no vetor de rotacao	##############################################
	to_send = np.array([255]+data_pelv[state].tolist()+[254], dtype=np.uint8)
	to_send2 = np.array([255]+data_foot[state].tolist()+[254], dtype=np.uint8)
	send_test = []

	#timers
	dTime = time.time() - start
	start = time.time()
	t += dTime
	t_fps += dTime
	t_state += dTime
	t_inercial += dTime

	#change state
	if(t_state >= frameRate):
		t_state = 0
		if state+1 == nEstados:
			perna = (perna+1)%2
		state = (state+1)%nEstados


	#fps calculator
	if t_fps > 1:
		os.system("clear")
		print ("fps:", fps)
		t_fps = 0
		fps = 0
	fps += 1
	

	#NANO (comunicacao) - luiz
	#t_inercial = 0
	#ser_uno.write('#')
	#qua = [mapeia(float(ord(c)),255.,360.) for c in ser_uno.readline()]
	#if(t_inercial*1000 > 20):
	''' qua = [float(ord(c))-90. for c in ser_uno.readline()]
	if len(qua) == 4:
		#t_inercial = 0
		incli[0] = 90. + qua[0]
		incli[1] = 90. + qua[1]
		iner = np.array(np.rint(incli), dtype=np.uint8)
		data_pelv[state][3] = iner[1]
		data_pelv[state][4] = iner[0]
		print data_pelv[state][3], " ", data_pelv[state][4]
	'''

	#STM (comunicacao) - simoes
	'''if s == 0xFE:
		s = spi.readbytes(8)
		if int(spi.readbytes(1)[0]) == 0xFD:
			#pode usar dados do s
			fps2 += 1
			print "RECEBEU: ",s
	spi.writebytes(to_send)
	'''

	#MEGA (comunicacao) marcos -teste
	if perna:
		send_test = np.array([255]+data_pelv[state].tolist()+data_foot[state].tolist()+[254], dtype=np.uint8)
		ser.write(''.join(str(chr(e)) for e in send_test))
	else:
		send_test = np.array([255]+data_foot[state].tolist()+data_pelv[state].tolist()+[254], dtype=np.uint8)
		ser.write(''.join(str(chr(e)) for e in send_test))
	#print state," --- ",send_test

	'''send_test = np.array([255]+data_pelv[state].tolist()+[254], dtype=np.uint8)
        send_test[1] = 60*math.sin(t_fps*2.*3.14/3) + 90
	output = ''.join(str(chr(e)) for e in send_test)
	#print send_test
	ser.write(output)
	#print ''.join(str(chr(e)) for e in send_test), "       ----        ", send_test
	res = ser.readline()
	if(len(res) == 12):
		adc = [int(ord(c))-90 for c in res]
		print adc[1:9]
	else:
		print res
	'''

#END +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
f.close()

