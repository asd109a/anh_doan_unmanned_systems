# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
def compute_index(min_x:int, max_x:int,min_y:int, max_y:int,gs:int,x_curr:int, y_curr:int)->int: 
    
    
    
    index= ((x_curr-min_x)/gs)+(((y_curr-min_y)/gs)*((max_x+gs-min_x)/gs))
    return index
min_x= 0
max_x= 10
gs=0.5
min_y= 0
max_y=10
x_curr=2
y_curr= 2

index= compute_index(min_x,max_x,min_y,max_y,gs,x_curr,y_curr)
print(index)