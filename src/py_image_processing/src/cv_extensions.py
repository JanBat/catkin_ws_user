
from robo_math import line_to_np
import cv2

#expects an array of lines, each of the format [[Y,X],[Y,X]](?) (each x,y pair being a point that forms the line)

def print_lines_onto_cv(lines, cv):
  for l in lines:
    
    #print(l)
    #cv2.line(cv,(l[0][0], l[1][0]),(l[2][0], l[3][0]),(255,0,0),5)
    cv2.line(cv,(l[0][1], l[0][0]),(l[1][1], l[1][0]),(255,0,0),1)
    #cv2.line(cv,(l[0][0], l[0][1]),(l[1][0], l[1][1]),(255,0,0),5)
    #cv2.line(cv,l[0],l[1],(255,0,0),5)
    

def make_top_half_all_black(cv):
    b,g,r = cv2.split(cv)
    for j in xrange(cv.shape[0]/2):
      for i in xrange(cv.shape[1]):
        r[j,i] = 0
        b[j,i] = 0
        g[j,i] = 0
    return cv2.merge((b,g,r))
          
def remove_all_non_grayscale(cv):
    b,g,r = cv2.split(cv)
    for j in xrange(cv.shape[0]):
      for i in xrange(cv.shape[1]):
        if (abs(r[j,i] - b[j,i])>40 or abs(g[j,i] - b[j,i])>40 or abs(r[j,i] - g[j,i])>40):
          r[j,i] = 0
          b[j,i] = 0
          g[j,i] = 0
    return cv2.merge((b,g,r))


def threshold(cv):
    b,g,r = cv2.split(cv)
    total = 0
    count = 0
    maximum = 0
    ts = 0.97   #anything darker than ts*maximum will be filtered out
    for j in xrange(cv.shape[0]):
      for i in xrange(cv.shape[1]):
        total = total +r[j,i]+b[j,i]+g[j,i] 
        count +=3
        maximum = max(maximum, r[j,i],b[j,i],g[j,i])
    average = (int)(total/count)  
    for j in xrange(cv.shape[0]):
      for i in xrange(cv.shape[1]):
        if r[j,i]< maximum*ts and b[j,i]< maximum*ts and g[j,i]< maximum*ts: 
          r[j,i] = 0
          b[j,i] = 0
          g[j,i] = 0

    return cv2.merge((b,g,r))

