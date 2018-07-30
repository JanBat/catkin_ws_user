
from robo_math import line_to_np
import cv2

#expects an array of lines, each of the format [[Y,X],[Y,X]](?) (each x,y pair being a point that forms the line)

def print_lines_onto_cv(lines, cv):
  for l in lines:
    
    #print(l)
    #cv2.line(cv,(l[0][0], l[1][0]),(l[2][0], l[3][0]),(255,0,0),5)
    cv2.line(cv,(l[0][1], l[0][0]),(l[1][1], l[1][0]),(255,0,0),5)
    #cv2.line(cv,(l[0][0], l[0][1]),(l[1][0], l[1][1]),(255,0,0),5)
    #cv2.line(cv,l[0],l[1],(255,0,0),5)
    
