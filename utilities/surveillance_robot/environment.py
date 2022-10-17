#!/usr/bin/env python

import random

class statement():
	def __init__(self, location, door):
		self.location = location
		self.door = door
		self.already_stated = 0
		self.delay = random.uniform(0.5,2)

	def stated(self):
		self.already_stated = 1

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