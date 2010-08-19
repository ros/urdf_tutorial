#!/usr/bin/python
import os
import sys
import subprocess

if __name__ == '__main__':
	if len(sys.argv) != 2:
		print "Usage: " + sys.argv[0] + " [directory]"
		sys.exit(-1)
	path= sys.argv[1]
	dirList=os.listdir(path)
	for fname in dirList:
		path1 = path + fname
		path2 = path + fname + "b"
		cmd = "rosrun ivcon ivcon " + path1 + " " + path2
		proc = subprocess.Popen([cmd], stdout=subprocess.PIPE, shell=True)
		(out, err) = proc.communicate()
		print err
