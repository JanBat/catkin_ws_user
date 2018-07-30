from math import sqrt

#adds 2 2d vectors
def add_vector_2D(v1, v2):
    return (v1[0]+v2[0], v1[1]+v2[1])

#subtracts 2 2d vectors
def sub_vector_2D(v1, v2):
    return (v1[0]-v2[0], v1[1]-v2[1])



#calculates (projected) intersection between two (non-parallel?) lines:
def intersection (line1, line2):
  factor = (-line2[0][0] * line1[1][1] + line2[0][0] * line1[0][1] + line1[0][0] * line1[1][1] - line1[0][0] * line1[0][1] + line2[0][1] * line1[1][0] - line2[0][1] * line1[0][0] - line1[0][1] * line1[1][0] + line1[0][1] * line1[0][0]) / (float)(line2[1][0] * line1[1][1] - line2[1][0] * line1[0][1] - line2[0][0] * line1[1][1] + line2[0][0] * line1[0][1] - line2[1][1] * line1[1][0] + line2[1][1] * line1[0][0] + line2[0][1] * line1[1][0] - line2[0][1] * line1[0][0])
  output_x = line2[0][0] + factor*(line2[1][0] - line2[0][0])
  output_y = line2[0][1] + factor*(line2[1][1] - line2[0][1])                                
  return (output_x, output_y)



#calculate distance between point and point
def distance_pp(point1, point2):
    direction = sub_vector_2D(point1, point2)
    #pythagoras:
    output = sqrt(direction[0]*direction[0]+direction[1]*direction[1])
    return output

#calculates distance between a line and a point
def distance_lp(line, point):
    #calculate direction vector of line:
    direction_x = line[0][0] - line[1][0]
    direction_y = line[0][1] - line[1][1]
    direction = (direction_x, direction_y)
    #calculate vector orthogonal to that direction vector
    orthogonal = (-direction_y, direction_x)   #just a direction vector
    #create line 2 from point to point+that orthogonal vector
    point_for_line2 = add_vector_2D(point, orthogonal)
    
    line2 = (point_for_line2, point)
    #calculate projected intersection between both lines
    intersect = intersection(line, line2)
    #calculate distance between point and intersection
    distance = distance_pp(intersect, point)
    #return that distance
    return distance

#before: [[1,1], [3,5]] (line from 1,1 to 3,5)
#after: [[2],[5],[1],[1]] (line from 1,1 with "direction" 2,5) 

def line_to_np(line):
  output = []
  output.append(line[1][0]-line[0][0])
  output.append(line[1][1]-line[0][1])
  output.append(line[0][0])
  output.append(line[0][1])
  return output




#reverse from line_to_np
def np_to_line(np):
  output = []
  output.append([(int)(np[2][0]-200*np[0][0]), (int)(np[3][0]-200*np[1][0])])
  output.append([(int)(np[2][0]+200*np[0][0]), (int)(np[3][0]+200*np[1][0])])
  return output






