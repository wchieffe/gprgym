from abc import ABC
from langchain.tools import StructuredTool
import zmq


class BaseSkill(ABC):
    def __init__(self):
        # TODO: Require description else raise NotImplementedError 
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://localhost:5558")


    def execute(self):
        raise NotImplementedError('Execute method must be implemented in subclass.')


    def as_tool(self):
        class_name = type(self).__name__
        # TODO: Add function arguments (e.g., xyz waypoints) aka zmq payload
        def tool_func():
            self.socket.send(bytes(class_name, 'utf-8'))
            message = self.socket.recv()
            return str(message)

        return StructuredTool.from_function(
            func = tool_func,
            name = class_name,
            description = self.description
        )
