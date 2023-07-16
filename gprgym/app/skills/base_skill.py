from abc import ABC

class BaseSkill(ABC):
    def __init__(self):
        self.name = "Base Skill"

    def get_name(self):
        return self.name
    
    def get_params(self):
        return
    
    def fire(self):
        return