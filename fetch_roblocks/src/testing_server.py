import socket
import time
import subprocess

if __name__ == '__main__':
	i = 1
	demo_process = None
	server = socket.socket()
	server.bind(("localhost", 12345))
	server.listen(5)
	while True:
		client,_ = server.accept()
		if i>1:
			subprocess.Popen(["killall -9 roscore"], shell=True)
			time.sleep(5)
		if demo_process:
			demo_process.kill()
			demo_process.terminate()
			demo_process.wait()
		demo_process = subprocess.Popen(["rosrun fetch_roblocks demo.py"], shell=True)
		time.sleep(20)
		i += 1
		if i == 3:
			server.close()
			break