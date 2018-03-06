import numpy as np 
import cv2

try:
    fp = open("points.txt", 'r')
    contents = fp.readlines()
finally:
    fp.close()
unexpected_chars = ['[',']','from','x','(',')','\n',',',';']
for line in contents:
	boxPoints = line.split()
