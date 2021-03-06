import RPi.GPIO as GPIO

LED = 18		# Pino do LED
PIN_IN = 25    # Pino do switch

# Configuração inicial
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED, GPIO.OUT)
GPIO.setup(PIN_IN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(LED,GPIO.LOW)

while True:
    if GPIO.input(PIN_IN):
        GPIO.output(LED,GPIO.HIGH)
    else:
        GPIO.output(LED,GPIO.LOW)
