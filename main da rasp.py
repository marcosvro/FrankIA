import os
from subprocess import check_output
import signal
import RPi.GPIO as GPIO
import time

PIN_OUT1 = 18	# Pino do primeiro STM32 ------ Pino 4 do STM32
PIN_OUT2 = 19	# Pino do segundo  STM32 ------ Pino 4 do STM32
PIN_IN = 20		# Pino de condição de inicio
LED = 21		# Pino do LED
TEMPO = 60		# Tempo de espera

def main():
#=======================	1	==========================================
	# Configuração inicial
	GPIO.setmode(GPIO.BCM)

	GPIO.setup(PIN_OUT1, GPIO.OUT)
	GPIO.setup(PIN_OUT2, GPIO.OUT)
	GPIO.setup(LED, GPIO.OUT)
	GPIO.setup(PIN_IN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	# Reseta os STM32
	GPIO.output(PIN_OUT1, GPIO.HIGH)	# TODO Verificar se o STM32 reseta em alto ou baixo
	GPIO.output(PIN_OUT2, GPIO.HIGH)

	# Espera apertar o botão ON
	while not GPIO.input(PIN_IN):
		time.sleep(1)
#========================	2	========================================
	# Contar até 60 segundos antes de começar o código
	final_time = time.time() + TEMPO
	while(time.time() < final_time):
		if not GPIO.input(PIN_IN): break
		time.sleep(1)
#=======================	3	========================================
	# Verifica se o pino de entrada está alto
	if GPIO.input(PIN_IN):
		# Inicia a caminhada
		os.system("python /home/pi/FrankIA/tragectory/extrator.py&")
		pid = int(check_output(["pidof","python"]).split()[0])
		# Camera process inicialize
		os.system("python ../visao/visao3.py&")
		cam_proc = int(check_output(["pidof", "python"]).split()[1])

		GPIO.output(LED,GPIO.HIGH)
	# Mantem a caminhada enquanto estiver en ON
	while GPIO.input(PIN_IN):
		time.sleep(1)
	GPIO.output(LED,GPIO.LOW)

	# Reseta os STM32
	GPIO.output(PIN_OUT1, GPIO.HIGH)	# TODO Verificar se o STM32 reseta em alto ou baixo
	GPIO.output(PIN_OUT2, GPIO.HIGH)

#=======================	4	========================================
	os.kill(pid, signal.SIGTERM)
	os.kill(cam_proc, signal.SIGTERM)
