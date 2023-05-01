import math
import numpy as np
import matplotlib.pyplot as plt

def exp_sat(x):
    c_o = 0.5
    tc = 5
    k = tc/c_o
    max_val = (1 - math.exp(-k*c_o))/(1+math.exp(-k*c_o))
    if(x>=c_o):
        res = 1
    elif(x<=-c_o):
        res = -1
    else:
        res = (1 - math.exp(-k*x))/(1+math.exp(-k*x))/max_val
    return res
    
if __name__ == '__main__':
    e_x = np.linspace(-1.0,1.0,1000)
    Kp = 4.0
    s_x = Kp * e_x
    results = []
    for i in range(len(s_x)):
        results.append(exp_sat(s_x[i]))
    plt.plot(e_x,results)
    plt.show()
    #mpldatacursor.datacursor()
