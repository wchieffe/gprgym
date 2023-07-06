import signal
import zmq

signal.signal(signal.SIGINT, signal.SIG_DFL)

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5558")

socket.send(b"move to")

message = socket.recv()
print(f"Received response: {message}")

socket.close()
context.term()