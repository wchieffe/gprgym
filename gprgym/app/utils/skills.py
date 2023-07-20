import importlib.util
import os

def load_skills(directory=None, parent=''):
    skills = []
    if directory == None:
        directory = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'skills')

    for filename in os.listdir(directory):
        path = os.path.join(directory, filename)
        module_name = parent + '.' + filename.split('.')[0]

        if os.path.isdir(path):
            # Recurse if it's a directory
            skills.extend(load_skills(path, module_name))
        elif filename.endswith('.py') and not filename.startswith('__'):
            # Load the module
            # TODO: Add error-handling once I start testing in VM
            module_spec = importlib.util.spec_from_file_location(module_name, path)
            module = importlib.util.module_from_spec(module_spec)
            module_spec.loader.exec_module(module) # TODO: Speed up

            # Get the class from the module
            class_name = filename.split('.')[0].replace("_", " ").title().replace(" ", "")
            skill_class = getattr(module, class_name)
            skills.append(skill_class())

    return skills
