import zmq

context = zmq.Context()

#  Socket to talk to server
print("Connecting to sim app server…")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

socket.send(b"pick")

socket.close()
context.term()