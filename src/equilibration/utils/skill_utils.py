import importlib.util
import logging
import os

def load_skills(directory=None, parent='skills') -> dict:
    """Load skill classes found in files throughout the \skills directory"""
    skills = {}
    if directory == None:
        directory = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'skills')

    for filename in os.listdir(directory):
        path = os.path.join(directory, filename)
        module_name = parent + '.' + filename.split('.')[0]

        if os.path.isdir(path):
            # Recurse if it's a directory
            skills.update(load_skills(path, module_name))
        elif filename.endswith('_skill.py') and not filename.startswith('base'):
            # Load the module
            print("module name: ", module_name)
            print("path name: ", path)
            module_spec = importlib.util.spec_from_file_location(module_name, path)
            module = importlib.util.module_from_spec(module_spec)
            module_spec.loader.exec_module(module) # TODO: Speed up

            # Get the class from the module
            class_name = filename.split('.')[0].replace("_", " ").title().replace(" ", "")
            print("class name: ", class_name)
            skill_class = getattr(module, class_name)
            skills[class_name] = skill_class()
            print("Success")

    logging.info("Tools added: ", skills.keys())

    return skills
