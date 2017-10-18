import numpy as np
import math
import threading
#import cv2
import time
import os
import spidev
import ikpy as ik
from ikpy import plot_utils

#v1 = 250 estados


#CONFIGS +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
deslocamentoZpes = 3.5
deslocamentoXpes = 5.
deslocamentoYpes = 0.
deslocamentoYpelves = 1.7


periodo = 20.
nEstados = 125
dMovx = deslocamentoXpes/nEstados
frameRate = periodo/nEstados
data_foot = np.zeros((nEstados,8), dtype=np.uint8)
data_pelv = np.zeros((nEstados,8), dtype=np.uint8)


#perna - quadril = target
link0 = ik.link.URDFLink("calc_lateral", [0,0, 0], [0,0,0], [1,0,0], use_symbolic_matrix=True, bounds=(-90,90))
link1 = ik.link.URDFLink("calc_frontal", [0,0, 0], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds=(-90,90))
link2 = ik.link.URDFLink("joelho", [0,0,8.24] , [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds=(-90,90))
link3 = ik.link.URDFLink("quadril", [0,0,6.45], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds=(-90,90))
#link4 = ik.link.URDFLink("pelves", [0, 1.7, 4], [0,	 0, 0], [1, 0, 0], use_symbolic_matrix=True, bounds=(-50,50))

#perna - pe = target
#link5 = ik.link.URDFLink("pelves", [0,0,0], [0,0,0], [1,0,0], use_symbolic_matrix=True, bounds = (-50, 50))
link6 = ik.link.URDFLink("quadril", [0, 0, 0], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds = (-180, 180))
link7 = ik.link.URDFLink("joelho", [0,0,-6.45], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds = (-180, 180))
link8 = ik.link.URDFLink("calc_frontal", [0,0,-8.24], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds = (-180, 180))
#link9 = ik.link.URDFLink("calc_lateral", [0,0,-4.48], [0,0,0], [1,0,0], use_symbolic_matrix=True, bounds = (-30, 30))
#link10 = ik.link.URDFLink("pe", [0,0,-2], [0,0,0], [0,0,0], use_symbolic_matrix=True)


#chains
foot2pelv = ik.chain.Chain([link0, link1, link2, link3], [True, True, True, False])
pelv2foot = ik.chain.Chain([link0, link1, link2, link3], [True, True, True, False])


#start joint positions
jointsf2p = np.deg2rad([0., 15.3, -35., 0.])
jointsp2f = np.deg2rad([0., 15.3, -35., 0.])
#jointsp2f = np.deg2rad([-5., 15., 0.])

"""
pos_test = foot2pelv.forward_kinematics(np.deg2rad([0.,23., -22.,  0.]))
print (pos_test[:3, 3])
exit()
"""



#start target position
pos_inicial_pelves = [3.33, 0., 14.]
pos_inicial_pe = [3.33, 0., 14.]

frame_target = np.eye(4)
frame_target[:3, 3] = pos_inicial_pelves
ik1 = foot2pelv.inverse_kinematics(frame_target,initial_position=jointsf2p)
frame_target2 = np.eye(4)
frame_target2[:3, 3] = pos_inicial_pe
ik2 = pelv2foot.inverse_kinematics(frame_target2, initial_position=jointsp2f)
jointsf2p = ik1;
jointsp2f = ik2;



#FUNCOES +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
'''Calcula cinematica inversa da pelves.   
	parm: indice(int) - diz em qual posicao do vetor de tragetoria deve ser armazenada a cinematica e qual momento da tragetoria calcular'''
def thread_cinematica_pelves(indice):
	pos = pos_inicial_pelves
	p = (deslocamentoXpes/2)*((np.exp((2*(indice-nEstados/2))/50) - np.exp((2*(indice-nEstados/2))/-50))/(np.exp((2*(indice-nEstados/2))/50)+np.exp((2*(indice-nEstados/2))/-50)))
	pos[0] = 0.5*p + 4.33
	pos[1] = -deslocamentoYpelves*np.sin(indice*np.pi/nEstados)
	frame_target = np.eye(4)
	frame_target[:3, 3] = pos
	lastpos = []
	

	ik = foot2pelv.inverse_kinematics(frame_target,initial_position=jointsf2p)
	ik = np.rad2deg(ik)

	roll = -ik[0]
	aux = 8.24*math.sin(np.deg2rad(ik[1]))
	aux = pos[0] - aux
	pitch = math.asin(aux/6.45)

	ik = ik.astype(np.int8)
	pitch = np.rad2deg(pitch).astype(np.int8)

	print (indice, " -- position: ", pos)


	#salva dados no array tragetoria
	#calc_lateral
	data_pelv[indice][0] = 90 + ik[0]
	#calc_frontal
	data_pelv[indice][1] = 90 + ik[1]
	#joelho
	data_pelv[indice][2] = 90 + ik[2]
	#quadril
	data_pelv[indice][3] = 90 + pitch
	#pelves
	data_pelv[indice][4] = 90 + roll
	#torso 
	data_pelv[indice][5] = 90
	#braco
	data_pelv[indice][6] = 90	
	#cotovelo	
	data_pelv[indice][7] = 90


'''Calcula cinematica inversa dos pes.
	parm: indice(int) - diz em qual posicao do vetor de tragetoria deve ser armazenada a cinematica e qual momento da tragetoria calcular'''
def thread_cinematica_pe(indice):
	pos = pos_inicial_pe
	pos[0] = 4.33 + 0.5*(-deslocamentoXpes/2)*((np.exp((2*(indice-nEstados/2))/50) - np.exp((2*(indice-nEstados/2))/-50))/(np.exp((2*(indice-nEstados/2))/50)+np.exp((2*(indice-nEstados/2))/-50)))
	pos[2] = 14. - deslocamentoZpes*np.exp(-((indice-nEstados/2)**2)/600)

	frame_target = np.eye(4)
	frame_target[:3, 3] = pos
	
	lastpos = []
	if(indice):
		last_pos = jointsp2f
	else:
		last_pos = [np.deg2rad(float(data_foot[indice-1][0]-90)),np.deg2rad(float(data_foot[indice-1][1]-90)),np.deg2rad(float(data_foot[indice-1][2]-90)),0.]
	
	ik = pelv2foot.inverse_kinematics(frame_target,initial_position=last_pos)
	ik = np.rad2deg(ik)

	roll = 2*(data_pelv[indice][0]-90)
	aux = 8.24*math.sin(np.deg2rad(ik[1]))
	aux = pos[0] - aux
	pitch = math.asin(aux/6.45)

	ik = ik.astype(np.int8)
	pitch = np.rad2deg(pitch).astype(np.int8)

	print (indice, " -- position: ", pos)

	#salva dados no array tragetoria
	#calc_lateral
	data_foot[indice][0] = 90 + roll
	#calc_frontal
	data_foot[indice][1] = 90 + ik[1]
	#joelho
	data_foot[indice][2] = 90 + ik[2]
	#quadril
	data_foot[indice][3] = 90 + pitch
	#pelves
	data_foot[indice][4] = 90 - roll
	#torso 
	data_foot[indice][5] = 90
	#braco
	data_foot[indice][6] = 90	
	#cotovelo	
	data_foot[indice][7] = 90


'''Cria nEstados threads para calcular a cinematica inversa considerando que o intervalo de execucao T esta particionado em nEstados.'''
def calculaTragetoria_pelves():
	i = 0
	while i < nEstados:
		#cria threads para calcular cinematica invesa dos pes
		thread = threading.Thread(target=thread_cinematica_pelves, args=(i, ))
		thread.daemon=True
		thread.start()
		thread.join()
		i += 1

def calculaTragetoria_pe():
	i = 0
	while i < nEstados:
		#cria threads para calcular cinematica invesa da pelves
		thread = threading.Thread(target=thread_cinematica_pe, args=(i, ))
		thread.daemon=True
		thread.start()
		thread.join()
		i += 1


#SETUP +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
print(np.rint(np.rad2deg(jointsf2p)))
print(np.rint(np.rad2deg(jointsp2f)))
#plot_chain(foot2pelv, juntas=jointsf2p)
#plot_chain(pelv2foot, juntas=jointsp2f)


iner = np.array([0., 0., 0., 0.], dtype=np.float)

#reading file
try:
	with open('data_pelv.txt', 'r') as f:
		data_pelv = np.loadtxt('data_pelv.txt').reshape((nEstados,8))
		print ("File data_pelv loaded!")
except IOError:
	print ("Calculando tragetoria.. ")
	calculaTragetoria_pelves()
	while threading.active_count() != 1:
		os.system("clear")
		print ("Calculando tragetoria.. (", threading.active_count(),"/",nEstados,")")
	np.savetxt('data_pelv.txt', data_pelv)

try:
	with open('data_foot.txt', 'r') as f:
		data_foot = np.loadtxt('data_foot.txt').reshape((nEstados,8))
		print ("File data_foot loaded!")	
except IOError:
	print ("Calculando tragetoria.. ")
	calculaTragetoria_pe()
	while threading.active_count() != 1:
		os.system("clear")
		print ("Calculando tragetoria.. (", threading.active_count(),"/",nEstados,")")
	np.savetxt('data_foot.txt', data_foot)



print (data_foot.shape)
print (data_pelv.shape)


#LOOP +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
start = time.time()
t = 0.
t_fps = 0.
t_state = 0.
t_inercial = 0.
state = 0
fps = 0

while 1:
	#sending data
	########################################	incluir iner no vetor de rotacao	##############################################
	#thread_cinematica_pe(693)
	to_send = [666]+[it - 90 for it in data_pelv[state]]+["---"]+[it - 90 for it in data_foot[state]]+[666]

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
		state = (state+1)%nEstados
		
	print (state, " -- ", to_send)
	
#END +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
f.close()

