#!/usr/bin/env python
from numpy import arcsin
import math

PI = 3.141592653

def vector2angle(vector):
  #print("before norm:", vector)
  n = normalize(vector)
  #print("after norm:", n)
  #arcsine of normalized.X:
  angle = arcsin(n[1])    #because with our cars, x axis goes through 0
  #print("after arcsin:", angle)
  #if x is negative, flip it over y-axis:
  if vector[0] < 0:
    if angle >= 0:
      angle = PI-angle
    else:
      angle = -PI - angle
  return angle
    



def normalize(vector):
  d = math.sqrt(vector[0]*vector[0] + vector[1] * vector[1])
  x = vector[0]/d
  y = vector[1]/d
  return [x,y]
