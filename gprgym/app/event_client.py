import zmq

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5558")

socket.send(b"hi")

message = socket.recv()
print(f"Received response: {message}")

socket.close()
context.term()