import numpy as np
import matplotlib.pyplot as plt


a=[1,2]
b=[3]

a+=[x+3 for x in b]
print(a)
print(b)