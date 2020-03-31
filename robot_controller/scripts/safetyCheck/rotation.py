import numpy as np
import math

def Rx(tx):
    Rx = np.array([ [   1,     0        ,   0          ,    0],
                    [   0, math.cos(tx) ,-math.sin(tx),     0],
                    [   0, math.sin(tx) , math.cos(tx),     0],
                    [   0,     0        ,   0          ,    1] ])
    return Rx

def Ry(ty):
    Ry = np.array([ [math.cos(ty)  , 0   , math.sin(ty),    0],
                    [   0          , 1   ,    0         ,   0],
                    [-math.sin(ty) , 0   , math.cos(ty),    0],
                    [   0          , 0   ,    0         ,   1] ])
    return Ry

def Rz(tz):
    Rz = np.array([ [math.cos(tz) ,-math.sin(tz) , 0,       0],
                    [math.sin(tz) , math.cos(tz) , 0,       0],
                    [   0         ,   0          , 1,       0],
                    [   0         ,   0          , 0,       1] ])
    return Rz
