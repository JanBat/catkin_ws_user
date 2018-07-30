from robo_math import distance_lp
from robo_math import line_to_np
from robo_math import np_to_line
from cv_extensions import print_lines_onto_cv
import cv2
import numpy

####ransac:
#expects a list/array of coordinates as such:[[0,0],[1,5],[123,456],...]
#returns a list of lines as such: [[[0,0],[1,1]], [[100,100], [0,50]],...]
#
#
#coordinates with values above 1 000 000 might not work
#does *NOT* eliminate outliers. filter those out first somewhere else!


def ransac (coordinates, linecount):
  #eventually return this:
  lines = []
  linecoords = []   #here, we save coords for each line

  # identify topleft and bottomright corner of the area that is populated with coordinates
  # initialize them with huge numbers (hopefully far away from all coordinates)
  topleft = [1000000,1000000] 
  bottomright = [-1000000,-1000000]
    
  for i in range(len(coordinates)):
        

    if coordinates[i][0] < topleft[0]:
      topleft[0] = coordinates[i][0]

    if coordinates[i][0] > bottomright[0]:
      bottomright[0] = coordinates[i][0]

    if coordinates[i][1] < topleft[1]:
      topleft[1] = coordinates[i][1]

    if coordinates[i][1] > bottomright[1]:
      bottomright[1] = coordinates[i][1]
  

  #size of the space the coordinates inhabit:
  #(several x/y swaps have happened here. :(
  
  width = bottomright[1] - topleft[1]
  height = bottomright[0] - topleft[0]
  #print("height/width:", height, width)
  # evenly distribute as many lines as "linecount" from topleft to bottomright
  for i in range(linecount):
    #make a line from top to bottom at topleft[0], topleft[0] + 1* width/linecount, 2* width/linecount etc.
    lateral = (int)((i+0.5)*width/linecount)
    #changed bottomright and topleft values here as well, something is clearly wrong  TODO: fix properly
    #points A and B that define the vertical line used for initialization:
    #DEBUG::::switched x/y coordinates here as well, something was fishy....
    #(seemed like the initial lines were internally horizontal, not vertical

    lineA = [topleft[0],topleft[1]+lateral]    #point at the top
    lineB = [bottomright[0],topleft[1]+lateral]#point at the bottom    

    
    line = [lineA, lineB]
    
    lines.append(line) #so we keep track of all lines from the get-go

    coords = []   #this will eventually contain all of the line's coords
    linecoords.append(coords)

  # now we have a list of lines ("lines") and a list of lists of coordinates ("linecoords")
  # the coords corresponding to lines[n] shall be put into linecoords[n]
   
  # #TODO: better iteration condition for ransac (?)
  for ransac_i in range(5):  
    
    #first, clear all the coordinates stored in linecoords. we need new ones in a bit.
    for l in range(len(linecoords)):
      linecoords[l] = []

    # foreach point in coordinates:
    for c in range(len(coordinates)):
      closestline = 0
      #iterate through lines + find closest one to coordinate c
      for l in range(len(lines)):
  
       if distance_lp(lines[l], coordinates[c]) < distance_lp(lines[closestline], coordinates[c]):    
         closestline = l
      
      #we iterated through all the lines and found the closest one. => add coord to linecoords[] accordingly:
      linecoords[closestline].append(coordinates[c])
      
    #linecoords now contains several separate lists, one for each line - 
    #fit each line to its coordinate list
    for i in range(linecount): 
      
      linecoord_np_array =numpy.array(linecoords[i])
      
      #some lines for debugging (show ransac iterations as pictures)
      #image = cv2.imread('img_bw.png')             #debug
      #for coord in linecoords[i]:                  #debug
      #   image[coord[0], coord[1]] = (0,255,0)     #debug
      #print lines[i]				   #debug
      #print (linecoord_np_array)                   #debug
      #print_lines_onto_cv([lines[i]], image)         #debug

      #fitline doesnt want an empty array. also, if that array is empty, line needs to be reset
      if len(linecoord_np_array) != 0:
          lines[i] = cv2.fitLine(linecoord_np_array, cv2.DIST_L2, 0, 0.01, 0.01)
      else:
          #hopefully this does it
          lines[i] = [[500],[500],[0],[0]]      



      #cv2.fitLine creates lines of format [[vx], [vy], [x], [y]] - we want [[x1,y1],[x2,y2]]
      lines[i] = np_to_line(lines[i])

      #name = 'ransac'+str(ransac_i)+'line'+str(i)+'.png'                  #debug
      #cv2.imwrite(name, image)                     #debug


  #done! (return lines)
  
  #print(lines)
  return lines
    
