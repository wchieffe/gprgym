from abc import ABC
from langchain.tools import StructuredTool
import zmq


class BaseSkill(ABC):
    def __init__(self):
        self.class_name = type(self).__name__
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://localhost:5558")
        if self.description == None:
            raise ValueError(f"Must define description in {self.class_name}.")


    def execute(self):
        raise NotImplementedError('Execute method must be implemented in subclass.')


    def as_tool(self):
        # TODO: Review this
        def tool_func(args: self.args_schema):
            self.socket.send(bytes(self.class_name, 'utf-8'))
            payload = {"skill_name": self.class_name, "args": args}
            self.socket.send_json(payload)
            message = self.socket.recv()
            return str(message)

        return StructuredTool.from_function(
            func = tool_func,
            name = self.class_name,
            description = self.description,
            args_schema=self.args_schema,
        )
