import numpy as np
import time
import math
import os
import serial
import socket
from subprocess import check_output
import signal
import struct


#CONFIGS +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
periodo = 0.2
nEstados = 125
frameRate = float(float(periodo)/float(nEstados))
data_foot = np.zeros((nEstados,8), dtype=np.uint8)
data_pelv = np.zeros((nEstados,8), dtype=np.uint8)
angulo_vira = 5
bussola = 0
meia_tela_pixel = 80.
meia_tela_angulo = 25.
faixa_erro_rotacao = 5


#Parametros de DH
L = np.array([4.5, 8.3, 6.45, 8.24, 0.], dtype=float)
frank_dir = [[np.pi/2., 0., -np.pi/2., 0., 0., 0., 0.],[-L[1], 0., 0., 0., 0., 0., 0.],[-L[0], 0., 0., L[2], L[3], 0., L[4]],[0., np.pi/2., -np.pi/2., 0., 0., np.pi/2., 0.]]
frank_esq = [[np.pi/2., 0., -np.pi/2., 0., 0., 0., 0.],[-L[1], 0., 0., 0., 0., 0., 0.],[L[0], 0., 0., L[2], L[3], 0., L[4]],[0., np.pi/2., -np.pi/2., 0., 0., np.pi/2., 0.]]
mat1 = np.array(frank_dir, dtype=float)
mat2 = np.array(frank_esq, dtype=float)

#AB_6
ab6 = np.array([[0.,0.,1.,0.],[0.,1.,0.,-4.5],[-1.,0.,0.,-22.99],[0.,0.,0.,1.]], dtype=float)


#COMUNICATION +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#serial
ser = serial.Serial('/dev/ttyUSB2', 115200, timeout=0)
ser2 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0)
ser_uno = serial.Serial('/dev/ttyUSB1', 230400, timeout=0)

#camera process inicialize
os.system("python ../visao/visao3.py&")
cam_proc = int(check_output(["pidof", "python"]).split()[0])

#socket
HOST = ''               # Endereco IP do Servidor
PORT = 2525             # Porta que o Servidor estax'
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
orig = (HOST, PORT)
udp.bind(orig)
udp.settimeout(0)



#FUNCTIONS +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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


def diferenca_angular(x):
	if(bussola-x > 0):
		dif = bussola - x
		if dif > 180:
			return dif - 360
		else:
			return dif
	else:
		dif = x - bussola
		if dif > 180:
			return 360 - dif
		else:
			return dif * -1
		

#SETUP +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#read tragetory file
try:
	with open('data_foot.txt', 'r') as f:
		data_foot = np.loadtxt('data_foot.txt').reshape((nEstados,8))
		print ("File data_foot loaded!")
	with open('data_pelv.txt', 'r') as f:
		data_pelv = np.loadtxt('data_pelv.txt').reshape((nEstados,8))
		print ("File data_pelv loaded!")
except IOError:
	print ("Error!! tragetory file not found..")
	exit()


#calculate turn tragetory
vira_pelv = np.array([0]*nEstados, dtype=np.int8)
vira_foot = np.array([0]*nEstados, dtype=np.int8)
for indice in range(nEstados):
	vira_pelv[indice] = angulo_vira + angulo_vira*((np.exp((2*(indice-nEstados/2))/50) - np.exp((2*(indice-nEstados/2))/-50))/(np.exp((2*(indice-nEstados/2))/50)+np.exp((2*(indice-nEstados/2))/-50)))
	vira_foot[indice] = angulo_vira - angulo_vira*((np.exp((2*(indice-nEstados/2))/50) - np.exp((2*(indice-nEstados/2))/-50))/(np.exp((2*(indice-nEstados/2))/50)+np.exp((2*(indice-nEstados/2))/-50)))


#read objetive direction
qua = []
while 1:
	buff = ser_uno.readline()
	if len(buff):
		qua = [float(int(c)-90) for c in buff]
	else:
		qua = []
	print ("Lendo direção objetivo..")
	if len(qua) != 6:
		continue
	else:
		break
if (qua[0]+90):
	bussola = qua[1] + 90 + 180
else:
	bussola = qua[1] + 90


#adjust joints direction
for i in range(nEstados):
	#data_pelv[i][0] = (data_pelv[i][0]-90)*-1 + 90
	data_foot[i][0] = (data_foot[i][0]-90)*-1 + 90
	data_pelv[i][2] = (data_pelv[i][2]-90)*-1 + 90
	data_foot[i][2] = (data_foot[i][2]-90)*-1 + 90
	data_pelv[i][3] = data_pelv[i][3] + 3
	data_foot[i][3] = data_foot[i][3] + 3
	#data_pelv[i][1] = 128
	data_foot[i][4] = 90
	#data_foot[i][4] = (data_foot[i][4]-90)*-1 + 90


#set paramters
start = time.time()
t = 0.
t_fps = 0.
t_state = 0.
t_inercial = 0.
state = 0
fps = 0
perna = 1
incli = [128.]*3
iner = np.array([0.]*3, dtype=np.uint8)
rot_desvio = 0
rot_real = 0
rota_esq = 0
rota_dir = 0
obstaculo = 0
pos_atual = []


#LOOP +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#test Juan -------------------------------------------------------------------------------
while 1:
	break
	dtime = time.time() - start
	start = time.time()
	t_fps += dtime

	if(t_fps >= 1.):
		os.system("clear")
		print ("fps: ", fps)
		t_fps = 0.
		fps = 0
	fps = fps + 1
	#fowa_cinematic(np.eye(4), mat1,6)
	T = back_cinematic(np.eye(4),mat2,0)
	invKinematic(T)

#main loop

try:
	while 1:
		send_test=[]
		#Timers
		dTime = time.time() - start
		start = time.time()
		t += dTime
		t_fps += dTime
		t_state += dTime
		t_inercial += dTime
	
	
		#Change state
		if t_state >= frameRate:
			t_state = 0
			if state+1 == nEstados:
				perna = (perna+1)%2			
				if math.fabs(rot_desvio) > 5:
					if rot_desvio < 0:
						if perna:
							rota_esq = -1
							rota_dir *= 2
						else:
							rota_dir = -1
							rota_esq *= 2
					else:
						if perna:
							rota_esq = 1
							rota_dir *= 2
						else:
							rota_dir = 1
							rota_esq *= 2
				else:
					if math.fabs(rota_esq) == 2:
						rota_esq = 0
					elif math.fabs(rota_esq) == 1:
						rota_esq *= 2					
					if math.fabs(rota_dir) == 2:
						rota_dir = 0
					elif math.fabs(rota_dir) == 1:
						rota_dir *= 2
				
			state = (state+1)%nEstados


		#FPS calculator
		if t_fps > 1:
			#os.system("clear")
			#print ("fps:", fps)
			t_fps = 0
			fps = 0
		fps += 1
	

		#posição read
		'''if(perna):
			buff2 = ser2.readline()
		else:
			buff2 = ser.readline()
		'''
		buff2 = ser.readline()
		qua2 = []
		if len(buff2):
			qua2 = [int(c)-90 for c in buff2]
		if len(qua2) == 10:
			pos_atual = np.array(np.rint(qua2), dtype=np.int)
			#print (pos_atual)
		ser.flushInput()
		#ser2.flushInput()


		#Inersial read (100hz)
		buff = ser_uno.readline()
		if len(buff):
			qua = [float(int(c)-90) for c in buff]
		else:
			qua = []	
		if len(qua) == 6:
			flag = qua[0]+90
			if flag:
				incli[2] = qua[1] + 90 + 180
			else:
				incli[2] = qua[1] + 90
			rot_real = incli[2]
			incli[0] =  qua[2] + 90
			incli[1] =  qua[3] + 90
			iner = np.array(np.rint(incli), dtype=np.uint8)
		

		#Low level write (bound rate)
		if perna:
			if rota_esq == 1:
				data_pelv[state][5] = 90 + vira_pelv[state]
			elif rota_esq == -1:
				data_pelv[state][5] = 90 + vira_pelv[state]*-1
			else:
				data_pelv[state][5] = 90

			if rota_dir == 2:
				data_foot[state][5] = 90 + vira_foot[state]
			elif rota_dir == -2:
				data_foot[state][5] = 90 + vira_foot[state]*-1		
			else:
				data_foot[state][5] = 90

			#print (data_foot[state][5], " -- vire ", rot_desvio, " graus")
			#pelv_iner = np.array([255]+data_pelv[state][:3].tolist()+iner[:2].tolist()+data_pelv[state][5:].tolist()+[254], dtype=np.uint8)
			send_pelv = np.array([255]+data_pelv[state].tolist()+[254], dtype=np.uint8)
			send_test = np.array([255]+data_foot[state].tolist()+[254], dtype=np.uint8)
			print (send_test, send_pelv)
			ser.write(struct.pack('>10B', *(send_test.tolist())))
			ser2.write(struct.pack('>10B', *(send_pelv.tolist())))
			#ser.write(struct.pack('>10B', 255, 90, 90, 90, 90, 90, 90, 90, 90, 254))
			#ser2.write(struct.pack('>10B', 255, 90, 90, 90, 90, 90, 90, 90, 90, 254))		
		else:
			if rota_dir == 1:
				data_pelv[state][5] = 90 + vira_pelv[state]
			elif rota_dir == -1:
				data_pelv[state][5] = 90 + vira_pelv[state]*-1
			else:
				data_pelv[state][5] = 90

			if rota_esq == 2:
				data_foot[state][5] = 90 + vira_foot[state]
			elif rota_esq == -2:
				data_foot[state][5] = 90 + vira_foot[state]*-1		
			else:
				data_foot[state][5] = 90
		
			#print (data_pelv[state][5], " -- vire ", rot_desvio, " graus")
			#pelv_iner = np.array([255]+data_pelv[state][:3].tolist()+iner[:2].tolist()+data_pelv[state][5:].tolist()+[254], dtype=np.uint8)
			send_pelv = np.array([255]+data_pelv[state].tolist()+[254], dtype=np.uint8)
			send_test = np.array([255]+data_foot[state].tolist()+[254], dtype=np.uint8)
			print (send_pelv, send_test)
			ser.write(struct.pack('>10B', *(send_pelv.tolist())))
			ser2.write(struct.pack('>10B', *(send_test.tolist())))
			#ser.write(struct.pack('>10B', 255, 90, 90, 90, 90, 90, 90, 90, 90, 254))
			#ser2.write(struct.pack('>10B', 255, 90, 90, 90, 90, 90, 90, 90, 90, 254))
				

		#Camera read (30hz)
		try:
			msg, cliente = udp.recvfrom(20)
			if len(msg) > 0 and int(msg) != 0:
				rot_desvio = float(int(msg))*meia_tela_angulo/meia_tela_pixel
				obstaculo = 1
			else:
				rot_desvio = diferenca_angular(rot_real)
				obstaculo = 0
		except BlockingIOError:
			if not obstaculo:
				rot_desvio = diferenca_angular(rot_real)
except KeyboardInterrupt:
    pass




#END +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
os.kill(cam_proc, signal.SIGTERM)
f.close()
udp.close()

