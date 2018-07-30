#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cv_extensions import print_lines_onto_cv


from ransac import ransac
#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt

# from __future__ import print_function


sys.setrecursionlimit(1500) #because why not

colorcode = [
(255,0,0),
(0,255,0),
(0,0,255),
(255,255,0),
(255,0,255),
(0,255,255),
(120,0,0),
(0,120,0),
(0,0,120),
(120,120,0),
(120,0,120),
(0,120,120),
(150,60,30),
(90,60,90)]



#reads a black and white file - entirely white pixels form the blobs
#return a picture (output[0]) and a list of blobs, each with coordinates (output[1])
def identify_blobs(data):
  #...
  
      
  

  b,g,r = cv2.split(data)
  
  bloblist = []
  #local function infect(i,j)
  #paints a cell black and adds its coordinates to the last blob in bloblist
  #does the same to neighbouring cells etc
  #resulting in painting an entire blob in 1 color
  def infect(i,j):
      if r[i,j] ==255: #should technically check for b and g as well
        #set color to length of bloblist - cause we're creating one blob at a time, so each blob gets its own slightly different color this way

        #coloring moved to separate part, once blobs are finished and shaved
        #color = len(bloblist*10) #*20 so it becomes visible in red

        #make it black - final blobs will be recolored
        r[i,j] = 0
        b[i,j] = 0
        g[i,j] = 0
        #add coordinate to blob - "bc" is a BlobCoordinate
        bc = [i,j]
        
        bloblist[len(bloblist)-1].append(bc)
        #now, recursively infect neighboring cells, if on the board
        #up
        if i < data.shape[0]-1:
          infect (i+1, j)
        #down
        if i > 0:
          infect (i-1, j)
        #left
        if j > 0:
          infect (i, j-1)
        #right
        if j < data.shape[1]-1:
          infect (i, j+1)
      #else:
          #do nothing

  #the same, but iterative - we had recursion limit problems
  def infect_iterative(i,j):
    infectionlist = []
    if r[i,j] ==255:
      
      infectionlist.append([i,j])
    while (len(infectionlist)>0):
      coord = infectionlist.pop()
      xx = coord[0]
      yy = coord[1]
      #color it black
      r[xx,yy] = 0
      b[xx,yy] = 0
      g[xx,yy] = 0
      #add to current blob:
      bloblist[len(bloblist)-1].append([xx,yy])
      #up
      if xx < data.shape[0]-1 and r[xx+1,yy]>0 :
        infectionlist.append([xx+1, yy])
      #down
      if xx > 0 and r[xx-1,yy]>0:
        infectionlist.append([xx-1, yy])
      #left
      if yy > 0 and r[xx,yy-1]>0:
        infectionlist.append([xx, yy-1])
      #right
      if yy < data.shape[1]-1 and r[xx,yy+1]>0:
        infectionlist.append([xx, yy+1])
       
  #end infect


    #try a blob infection on every cell of the entire image. 
    #black cells and already infected cells (by a different blob) will be spared
  for i in range(data.shape[0]):
    for j in range(data.shape[1]): 
      if r[i,j] == 255:   #we're assuming all blobbable points are white
          #we doublechecked r[i,j] for 255 to only add new blobs here
        blob = []
        bloblist.append(blob)   
        infect_iterative(i,j)	
	  
  #shave blobs
  #HAC: the "4" below assumes that we the 4 biggest blobs on the input picture are made up of 
  #4 lines on the street (left, right, 2 small ones in the middle
  while len(bloblist)>10:     #make this more interactive
      #delete smallest blob
      smallest_blob = 0
      for i in range(len(bloblist)):
        if (len(bloblist[i]) < len(bloblist[smallest_blob])):
          smallest_blob = i        
      del bloblist[smallest_blob]

  #print bloblist 
  #print "(bloblist after shave)"  
  #calculate middlepoints   #TODO: extract this as a function
  blob_middlepoint_list = []
  for i in range(len(bloblist)):
    total = [0,0]
    for j in range(len(bloblist[i])):
      total = (total[0]+bloblist[i][j][0], total[1]+bloblist[i][j][1])
    total = (total[0]/len(bloblist[i]), total[1]/len(bloblist[i]))
    blob_middlepoint_list.append(total)
    #done!
  
  
  #recolor blobs, each with their own color:
  for i in range(len(bloblist)):
    blob = bloblist[i]
    for j in range(len(blob)):
      r[blob[j][0],blob[j][1]], b[blob[j][0],blob[j][1]], g[blob[j][0],blob[j][1]] = colorcode[i][0],colorcode[i][1],colorcode[i][2]  
  

  
  #output has two components:
  #[0]: cv2image with colored-in blobs
  #[1]: list of blobs, each a list of coordinates

  output = []
  output.append(cv2.merge((b,g,r)))
  output.append(bloblist)
  return output

#main part:
#(testing)



