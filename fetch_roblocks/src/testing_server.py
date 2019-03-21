import socket
import time
import subprocess

if __name__ == '__main__':
	i = 1
	demo_process, message_publish = None, None
	server = socket.socket()
	server.bind(("localhost", 12345))
	server.listen(5)
	while True:
		if i>1:
			subprocess.Popen(["killall -9 roscore"], shell=True)
			time.sleep(5)
		if demo_process:
			demo_process.kill()
			demo_process.terminate()
			demo_process.wait()
		demo_process = subprocess.Popen(["rosrun fetch_roblocks demo.py"], shell=True)
		time.sleep(20)
		client,_ = server.accept()
		message_publish = subprocess.Popen(["rostopic pub /ready_msg fetch_roblocks/Ready 1"], shell=True)
		client,_ = server.accept()
		message_publish.kill()
		message_publish.terminate()
		message_publish.wait()
		i += 1