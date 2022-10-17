#!/usr/bin/env python
import rospy
import random
import time

from surveillance_robot import environment as env
from surveillance_robot import architecture_name_mapper as anm

def main():
	random.shuffle(env.statement_list)
	for stat in env.statement_list:
		if stat.already_stated == 0:
			time.sleep(stat.delay)
			print('Door '+stat.door+' is in location '+stat.location)
			stat.stated()


if __name__ == '__main__':
    main()