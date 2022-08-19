import csv
import numpy as np
import matplotlib.pyplot as plt

x=np.linspace(-5,5,5000)
y=1/4 * np.log(1 + np.exp(4 * x))


plt.plot(x,y,'b')
plt.show()
# plt.savefig("./figs/long jump/electrical_work_5_inputs")

    
    