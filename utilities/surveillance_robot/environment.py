#!/usr/bin/env python
"""
.. module:: environment
  :platform: Unix 
  :synopsis: Python module that simulates the environment
.. moduleauthor:: Ettore Sani 5322242@studenti.unige.it

This module stores the knowledge about the environment that the :mod:`find_qr` module discovers.

"""

### CLASSES ###
class statement():
	"""This class represents a statement of the environment.

	"""
	def __init__(self, location, door):
		self.location = location
		self.door = door
		self.already_stated = 0

	def stated(self):
		"""Function to call when the statement have been stated.

		"""
		self.already_stated = 1

### GLOBAL ###
# List of statements about the environment.
# Change this list to represent different environments.
statement_list = [  statement('E','D6'),
					statement('E','D7'),
					statement('C1','D6'),
					statement('C1','D1'),
					statement('C1','D5'),
					statement('C1','D2'),
					statement('R1','D1'),
					statement('R2','D2'),
					statement('C2','D5'),
					statement('C2','D4'),
					statement('C2','D3'),
					statement('C2','D7'),
					statement('R3','D3'),
					statement('R4','D4'),
				 	]