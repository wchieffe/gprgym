from abc import ABC
from langchain.tools import StructuredTool
import logging
import zmq


class BaseSkill(ABC):
    def __init__(self):
        self.class_name = type(self).__name__
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://localhost:5558")

        if self.description == None:
            raise ValueError(f"Must define description in {self.class_name}.")

        # args_schema should be defined in the inherited class unless there are no args
        try:
            self.args_schema = self.args_schema
        except AttributeError:
            logging.warn(f"Best practice is to define args_schema in {self.class_name} before calling super().__init__()")
            logging.warn(f"If there are no args, specify self.args_schema = None.")
            self.args_schema = None


    def execute(self, args, scene):
        raise NotImplementedError('Execute method must be implemented in subclass.')


    def as_tool(self):
        def tool_func(args: self.args_schema = None):
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