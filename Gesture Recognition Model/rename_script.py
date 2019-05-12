import os

cwdir = os.getcwd()
origin = cwdir+"//<Your Project Directory>//augmented//left//" 	  # source directory
destination = cwdir+"//<Your Project Directory>//augmented//left_final//"	# destination directory 

for i, filename in enumerate(os.listdir('augmented//left')):
    os.rename(origin + filename, destination + 'l' + str(i).zfill(4) + ".jpg")

