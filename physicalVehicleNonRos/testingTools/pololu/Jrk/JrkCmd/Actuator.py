''' script to communicate with Jrk 21v3 using JrkCmd  '''

import subprocess
import time 

def getFeedback():
	
	command = "./JrkCmd" + " -s"
	subprocess.call(command, stdout = open( 'output.txt', 'w'), shell=True)
	
	# process the output to get the feedback
	# open the file
	file = open('output.txt', 'r')
	variables = file.readlines()
	print(variables)
	
	# read the feedback
	feedback = variables[5].split()[1]
	print(feedback)
	
	# close and empty the file
	open('output.txt', 'w').close() 
	
	return feedback



def setTarget(target):
	
	command = "./JrkCmd" + " " + "--target" + " " + str(target)
	print("Args: " + command)
	subprocess.call(command, shell=True)
	


if __name__ == '__main__':

	target = 1600
	setTarget(target)
	
	#time.sleep(2)
	
	val = getFeedback()
	print("Feedback: " + str(val))
	
	
	
'''
	# by Chris, on Windows
	# 1550 right
	# 1829 middle for the steering
	# 2276 left
	
	# returned by feedback
	# 2116: left
	# 1695: right
	# 1850: middle 
	
'''
	
	
	
	



