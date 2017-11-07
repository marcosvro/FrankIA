import os
from subprocess import check_output
import signal
import RPi.GPIO as GPIO
import time

PIN_RESET = 18	# Pino do primeiro STM32 ------ Pino 4 do STM32
PIN_IN = 20		# Pino de condição de inicio
LED = 21		# Pino do LED
TEMPO = 5		# Tempo de espera em segundos
TEMPO_STM = 0.2 # Tempo que espera até o STM32 resetar

while True:
	# Configuração inicial
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(PIN_RESET, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
	GPIO.setup(LED, GPIO.OUT)
	GPIO.setup(PIN_IN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
	# Reseta os STM32
	GPIO.output(PIN_RESET, GPIO.HIGH)
	time.sleep(TEMPO_STM)	#200 ms
	# Liga os STM32
	GPIO.output(PIN_RESET, GPIO.LOW)

	# Espera apertar o botão ON
	while not GPIO.input(PIN_IN):
		time.sleep(1)
	# Contar até 60 segundos antes de começar o código
	final_time = time.time() + TEMPO
	while(time.time() < final_time):
		if not GPIO.input(PIN_IN):
			break
		time.sleep(1)
	# Verifica se o pino de entrada está alto
	if GPIO.input(PIN_IN):
		# Inicia a caminhada
		os.system("python /home/pi/FrankIA/tragectory/extrator.py&")
		pid = int(check_output(["pidof","python"]).split()[0])
		# Camera process inicialize
		os.system("python ../visao/visao3.py&")
		cam_proc = int(check_output(["pidof", "python"]).split()[0])
		GPIO.output(LED,GPIO.HIGH)
	# Mantem a caminhada enquanto estiver en ON
	while GPIO.input(PIN_IN):
		time.sleep(1)
	GPIO.output(LED,GPIO.LOW)

	# Reseta os STM32
	GPIO.output(PIN_RESET, GPIO.HIGH)	# TODO Verificar se o STM32 reseta em alto ou baixo

	os.kill(pid, signal.SIGTERM)
	os.kill(cam_proc, signal.SIGTERM)
