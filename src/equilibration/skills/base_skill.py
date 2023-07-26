from abc import ABC
import json
import logging
try:
    # TODO: Better way to ignore import errors when sim controller is loading the skills
    from langchain.tools import StructuredTool
except:
    pass


class BaseSkill(ABC):
    def __init__(self):
        self.class_name = type(self).__name__

        try:
            assert self.description != None
        except:
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


    def as_tool(self, socket):
        def tool_func(**kwargs):
            payload = {"skill_name": self.class_name}
            if kwargs:
                payload["args"] = json.dumps(kwargs)
            else:
                payload["args"] = None
            socket.send_json(payload)
            message = socket.recv()
            return str(message)

        return StructuredTool.from_function(
            func = tool_func,
            name = self.class_name,
            description = self.description,
            args_schema=self.args_schema,
        )
