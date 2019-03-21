import socket

if __name__ == '__main__':
	while True:
		choice = int(input('Enter your choice'))
		if choice == 1:
			client = socket.socket()
			client.connect(('localhost', 12345))
			client.close()
		elif choice == 0:
			break