#!/usr/bin/env python3
from pathlib import Path
import inspect
import importlib
from sys import stdout

# Converts from string format "ThisIsAString" to "this_is_a_string"
def module_name(string):

    pythonic_string = ""

    for i in range(len(string)):

        letter = string[i:i+1]

        if letter.isupper():
            letter = letter.lower()

            if i>0:
                letter = "_" + letter
        pythonic_string += letter

    return pythonic_string


# Assumes 1 class per file with the same naming protocol. Imports the first class in
# the specified module
def import_node(node_class_name):

    node_name = module_name(node_class_name)

    node_Path = list(Path("nodes").rglob(node_name + ".py"))


    node_filepath = str(node_Path[0])

    node_module_name = node_filepath.replace("/", ".").replace(".py", "")

    node_module = importlib.import_module(node_module_name)
    node_class =  getattr(node_module, node_class_name)


    return node_class



