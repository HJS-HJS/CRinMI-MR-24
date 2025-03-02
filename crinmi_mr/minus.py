import numpy as np
workspace = {
    "l_t" : [-0.19095917, -0.82109088, 0.27374902], 
    "l_m" : [-0.19671763, -0.67092103, 0.27810538], 
    "l_b" : [-0.19563674, -0.52807312, 0.28605536], 
    "r_t" : [-0.42820886, -0.82423891, 0.26943872], 
    "r_m" : [-0.43223348, -0.67932685, 0.2704712],  
    "r_b" : [-0.43613321, -0.53366284, 0.27435081], 
}

measure = {
    "l_t" : [-0.19905, -0.84166, 0.30266],
    "l_m" : [-0.19889, -0.69427, 0.29972],
    "l_b" : [-0.19156, -0.55399, 0.29743],
    "r_t" : [-0.42735, -0.83609, 0.30496],
    "r_m" : [-0.42523, -0.69363, 0.30262],
    "r_b" : [-0.42070, -0.55359, 0.30120],
}

t_list = ["l_t","l_m","l_b","r_t","r_m","r_b"]
for i in t_list:
  print(i,":", np.array(measure[i]) -np.array(workspace[i] - np.array([0, 0, 0.01])))



