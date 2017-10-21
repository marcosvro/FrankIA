import socket


HOST = ''              # Endereco IP do Servidor
PORT = 666             # Porta que o Servidor esta
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
orig = (HOST, PORT)
udp.bind(orig)


while 1:
	msg, cliente = udp.recvfrom(20)
	print (cliente, int(msg))
