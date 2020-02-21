#! /usr/bin/env micropython

class Field:

    # Constructors
    def __init__(self):
        self.width = 40         # Create Initial Width and Height conditions
        self.height = 200

        self.grid = []          # Create initial grid bit