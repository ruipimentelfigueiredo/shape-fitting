#
#  Copyright (C) 2018 Rui Pimentel de Figueiredo
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#  
#      http://www.apache.org/licenses/LICENSE-2.0
#      
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#
#    
#    \author Rui Figueiredo : ruipimentelfigueiredo
#



#! /usr/bin/env python
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
from os import path

#plt.rc('text', usetex=True)
import numpy as np

import math
home=path.expanduser('~/')

def to_percent(y, position):
    # Ignore the passed in position. This has the effect of scaling the default
    # tick locations.
    s = str(100 * y)

    # The percent symbol needs escaping in latex
    if plt.rcParams['text.usetex'] is True:
        return s #+ r'$\%$'
    else:
        return s #+ '%'

heights=5
radii=1
iterations=500
ground_truth_size=heights*radii
outlier_levels=9
noise_levels_number=11
noise_index=0
outlier_index=0
occlusion_index=0
alpha_=0.1
fontsize_=20
error_levels=[0,5,10,15,20,25,30,35,40,45,50]
noise_levels_number=len(error_levels)
outlier_levels_=[0,25,50,75,100,125,150,175,200]
occlusion_levels_=[0.0]
occlusion_levels_number=1

colors=['green','blue','red','black','orange','orange','aqua','yellow']
labels=['Rabbani et al.','Rabbani et al. (soft-voting)','Ours (Unbiased)','Ours (Unbiased and soft-voting)','Ours (Weak Vertical-Bias)','Ours (Weak Vertical-Bias and soft-voting)','Ours (Strong Vertical-Bias)','Ours (Strong Vertical-Bias and soft-voting)']
linestyles = ['-', '--']
linethickness=[1, 2, 3, 4, 5]
#ORIENTATION
hough_orientation_results_0=[]
hough_orientation_results_1=[]
hough_orientation_results_2=[]
hough_orientation_results_3=[]
hough_orientation_results_4=[]
hough_orientation_results_5=[]
hough_orientation_results_6=[]
hough_orientation_results_7=[]

hough_orientation_file_0 = open(home + "shape-fitting/dataset/results/orientation_noise_0.txt", "r") 
hough_orientation_file_1 = open(home + "shape-fitting/dataset/results/orientation_noise_1.txt", "r") 
hough_orientation_file_2 = open(home + "shape-fitting/dataset/results/orientation_noise_2.txt", "r") 
hough_orientation_file_3 = open(home + "shape-fitting/dataset/results/orientation_noise_3.txt", "r") 
hough_orientation_file_4 = open(home + "shape-fitting/dataset/results/orientation_noise_4.txt", "r") 
hough_orientation_file_5 = open(home + "shape-fitting/dataset/results/orientation_noise_5.txt", "r") 
hough_orientation_file_6 = open(home + "shape-fitting/dataset/results/orientation_noise_6.txt", "r") 
hough_orientation_file_7 = open(home + "shape-fitting/dataset/results/orientation_noise_7.txt", "r") 

for line in hough_orientation_file_0:
  hough_orientation_results_0.append(180.0*float(line)/math.pi)
hough_orientation_results_0 = np.array(hough_orientation_results_0).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_orientation_file_1:
  hough_orientation_results_1.append(180.0*float(line)/math.pi)
hough_orientation_results_1 = np.array(hough_orientation_results_1).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_orientation_file_2:
  hough_orientation_results_2.append(180.0*float(line)/math.pi)
hough_orientation_results_2 = np.array(hough_orientation_results_2).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_orientation_file_3:
  hough_orientation_results_3.append(180.0*float(line)/math.pi)
hough_orientation_results_3 = np.array(hough_orientation_results_3).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_orientation_file_4:
  hough_orientation_results_4.append(180.0*float(line)/math.pi)
hough_orientation_results_4 = np.array(hough_orientation_results_4).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_orientation_file_5:
  hough_orientation_results_5.append(180.0*float(line)/math.pi)
hough_orientation_results_5 = np.array(hough_orientation_results_5).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)


for line in hough_orientation_file_6:
  hough_orientation_results_6.append(180.0*float(line)/math.pi)
hough_orientation_results_6 = np.array(hough_orientation_results_6).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)


for line in hough_orientation_file_7:
  hough_orientation_results_7.append(180.0*float(line)/math.pi)
hough_orientation_results_7 = np.array(hough_orientation_results_7).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)


#POSITION
hough_position_results_0=[]
hough_position_results_1=[]
hough_position_results_2=[]
hough_position_results_3=[]
hough_position_results_4=[]
hough_position_results_5=[]
hough_position_results_6=[]
hough_position_results_7=[]

hough_position_file_0 = open(home + "shape-fitting/dataset/results/position_noise_0.txt", "r") 
hough_position_file_1 = open(home + "shape-fitting/dataset/results/position_noise_1.txt", "r") 
hough_position_file_2 = open(home + "shape-fitting/dataset/results/position_noise_2.txt", "r") 
hough_position_file_3 = open(home + "shape-fitting/dataset/results/position_noise_3.txt", "r") 
hough_position_file_4 = open(home + "shape-fitting/dataset/results/position_noise_4.txt", "r") 
hough_position_file_5 = open(home + "shape-fitting/dataset/results/position_noise_5.txt", "r") 
hough_position_file_6 = open(home + "shape-fitting/dataset/results/position_noise_6.txt", "r") 
hough_position_file_7 = open(home + "shape-fitting/dataset/results/position_noise_7.txt", "r") 

for line in hough_position_file_0:
  hough_position_results_0.append(float(line))
hough_position_results_0 = np.array(hough_position_results_0).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_position_file_1:
  hough_position_results_1.append(float(line))
hough_position_results_1 = np.array(hough_position_results_1).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_position_file_2:
  hough_position_results_2.append(float(line))
hough_position_results_2 = np.array(hough_position_results_2).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_position_file_3:
  hough_position_results_3.append(float(line))
hough_position_results_3 = np.array(hough_position_results_3).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_position_file_4:
  hough_position_results_4.append(float(line))
hough_position_results_4 = np.array(hough_position_results_4).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_position_file_5:
  hough_position_results_5.append(float(line))
hough_position_results_5 = np.array(hough_position_results_5).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_position_file_6:
  hough_position_results_6.append(float(line))
hough_position_results_6 = np.array(hough_position_results_6).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_position_file_7:
  hough_position_results_7.append(float(line))
hough_position_results_7 = np.array(hough_position_results_7).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)


#RADIUS
hough_radius_results_0=[]
hough_radius_results_1=[]
hough_radius_results_2=[]
hough_radius_results_3=[]
hough_radius_results_4=[]
hough_radius_results_5=[]
hough_radius_results_6=[]
hough_radius_results_7=[]

hough_radius_file_0 = open(home + "shape-fitting/dataset/results/radius_noise_0.txt", "r") 
hough_radius_file_1 = open(home + "shape-fitting/dataset/results/radius_noise_1.txt", "r") 
hough_radius_file_2 = open(home + "shape-fitting/dataset/results/radius_noise_2.txt", "r") 
hough_radius_file_3 = open(home + "shape-fitting/dataset/results/radius_noise_3.txt", "r") 
hough_radius_file_4 = open(home + "shape-fitting/dataset/results/radius_noise_4.txt", "r") 
hough_radius_file_5 = open(home + "shape-fitting/dataset/results/radius_noise_5.txt", "r") 
hough_radius_file_6 = open(home + "shape-fitting/dataset/results/radius_noise_6.txt", "r") 
hough_radius_file_7 = open(home + "shape-fitting/dataset/results/radius_noise_7.txt", "r") 

for line in hough_radius_file_0:
  hough_radius_results_0.append(float(line))
hough_radius_results_0 = np.array(hough_radius_results_0).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_radius_file_1:
  hough_radius_results_1.append(float(line))
hough_radius_results_1 = np.array(hough_radius_results_1).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_radius_file_2:
  hough_radius_results_2.append(float(line))
hough_radius_results_2 = np.array(hough_radius_results_2).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_radius_file_3:
  hough_radius_results_3.append(float(line))
hough_radius_results_3 = np.array(hough_radius_results_3).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_radius_file_4:
  hough_radius_results_4.append(float(line))
hough_radius_results_4 = np.array(hough_radius_results_4).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_radius_file_5:
  hough_radius_results_5.append(float(line))
hough_radius_results_5 = np.array(hough_radius_results_5).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_radius_file_6:
  hough_radius_results_6.append(float(line))
hough_radius_results_6 = np.array(hough_radius_results_6).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_radius_file_7:
  hough_radius_results_7.append(float(line))
hough_radius_results_7 = np.array(hough_radius_results_7).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)


# compute orientation average and standard deviation
hough_orientation_results_mean_0 = np.mean(hough_orientation_results_0, axis=(0,3))
hough_orientation_results_std_0  = np.std(hough_orientation_results_0, axis=(0,3))

hough_orientation_results_mean_1 = np.mean(hough_orientation_results_1, axis=(0,3))
hough_orientation_results_std_1  = np.std(hough_orientation_results_1, axis=(0,3))

hough_orientation_results_mean_2 = np.mean(hough_orientation_results_2, axis=(0,3))
hough_orientation_results_std_2  = np.std(hough_orientation_results_2, axis=(0,3))

hough_orientation_results_mean_3 = np.mean(hough_orientation_results_3, axis=(0,3))
hough_orientation_results_std_3  = np.std(hough_orientation_results_3, axis=(0,3))

hough_orientation_results_mean_4 = np.mean(hough_orientation_results_4, axis=(0,3))
hough_orientation_results_std_4  = np.std(hough_orientation_results_4, axis=(0,3))

hough_orientation_results_mean_5 = np.mean(hough_orientation_results_5, axis=(0,3))
hough_orientation_results_std_5  = np.std(hough_orientation_results_5, axis=(0,3))

hough_orientation_results_mean_6 = np.mean(hough_orientation_results_6, axis=(0,3))
hough_orientation_results_std_6  = np.std(hough_orientation_results_6, axis=(0,3))

hough_orientation_results_mean_7 = np.mean(hough_orientation_results_7, axis=(0,3))
hough_orientation_results_std_7  = np.std(hough_orientation_results_7, axis=(0,3))

# compute position average and standard deviation
hough_position_results_mean_0 = np.mean(hough_position_results_0, axis=(0,3))
hough_position_results_std_0  = np.std(hough_position_results_0, axis=(0,3))

hough_position_results_mean_1 = np.mean(hough_position_results_1, axis=(0,3))
hough_position_results_std_1  = np.std(hough_position_results_1, axis=(0,3))

hough_position_results_mean_2 = np.mean(hough_position_results_2, axis=(0,3))
hough_position_results_std_2  = np.std(hough_position_results_2, axis=(0,3))

hough_position_results_mean_3 = np.mean(hough_position_results_3, axis=(0,3))
hough_position_results_std_3  = np.std(hough_position_results_3, axis=(0,3))

hough_position_results_mean_4 = np.mean(hough_position_results_4, axis=(0,3))
hough_position_results_std_4  = np.std(hough_position_results_4, axis=(0,3))

hough_position_results_mean_5 = np.mean(hough_position_results_5, axis=(0,3))
hough_position_results_std_5  = np.std(hough_position_results_5, axis=(0,3))

hough_position_results_mean_6 = np.mean(hough_position_results_6, axis=(0,3))
hough_position_results_std_6  = np.std(hough_position_results_6, axis=(0,3))

hough_position_results_mean_7 = np.mean(hough_position_results_7, axis=(0,3))
hough_position_results_std_7  = np.std(hough_position_results_7, axis=(0,3))

# compute radius average and standard deviation
hough_radius_results_mean_0 = np.mean(hough_radius_results_0, axis=(0,3))
hough_radius_results_std_0  = np.std(hough_radius_results_0, axis=(0,3))

hough_radius_results_mean_1 = np.mean(hough_radius_results_1, axis=(0,3))
hough_radius_results_std_1  = np.std(hough_radius_results_1, axis=(0,3))

hough_radius_results_mean_2 = np.mean(hough_radius_results_2, axis=(0,3))
hough_radius_results_std_2  = np.std(hough_radius_results_2, axis=(0,3))

hough_radius_results_mean_3 = np.mean(hough_radius_results_3, axis=(0,3))
hough_radius_results_std_3  = np.std(hough_radius_results_3, axis=(0,3))

hough_radius_results_mean_4 = np.mean(hough_radius_results_4, axis=(0,3))
hough_radius_results_std_4  = np.std(hough_radius_results_4, axis=(0,3))

hough_radius_results_mean_5 = np.mean(hough_radius_results_5, axis=(0,3))
hough_radius_results_std_5  = np.std(hough_radius_results_5, axis=(0,3))

hough_radius_results_mean_6 = np.mean(hough_radius_results_6, axis=(0,3))
hough_radius_results_std_6  = np.std(hough_radius_results_6, axis=(0,3))

hough_radius_results_mean_7 = np.mean(hough_radius_results_7, axis=(0,3))
hough_radius_results_std_7  = np.std(hough_radius_results_7, axis=(0,3))

### Plots (noise)

### Orientation
plt.figure(figsize=(8, 6))
plt.plot(error_levels,hough_orientation_results_mean_0[outlier_index,occlusion_index,:],color=colors[0],label=labels[0],linestyle=linestyles[1])
error_sup=hough_orientation_results_mean_0[outlier_index,occlusion_index,:]+hough_orientation_results_std_0[outlier_index,occlusion_index,:];
error_inf=hough_orientation_results_mean_0[outlier_index,occlusion_index,:]-hough_orientation_results_std_0[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0],linestyle=linestyles[1])
plt.ylim((0, 50))
plt.plot(error_levels,hough_orientation_results_mean_1[outlier_index,occlusion_index,:],color=colors[0],label=labels[1],linestyle=linestyles[0])
error_sup=hough_orientation_results_mean_1[outlier_index,occlusion_index,:]+hough_orientation_results_std_1[outlier_index,occlusion_index,:];
error_inf=hough_orientation_results_mean_1[outlier_index,occlusion_index,:]-hough_orientation_results_std_1[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0],linestyle=linestyles[0])

plt.plot(error_levels,hough_orientation_results_mean_2[outlier_index,occlusion_index,:],color=colors[1],label=labels[2],linestyle=linestyles[1])
error_sup=hough_orientation_results_mean_2[outlier_index,occlusion_index,:]+hough_orientation_results_std_2[outlier_index,occlusion_index,:];
error_inf=hough_orientation_results_mean_2[outlier_index,occlusion_index,:]-hough_orientation_results_std_2[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(error_levels,hough_orientation_results_mean_3[outlier_index,occlusion_index,:],color=colors[1],label=labels[3],linestyle=linestyles[0])
error_sup=hough_orientation_results_mean_3[outlier_index,occlusion_index,:]+hough_orientation_results_std_3[outlier_index,occlusion_index,:];
error_inf=hough_orientation_results_mean_3[outlier_index,occlusion_index,:]-hough_orientation_results_std_3[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(error_levels,hough_orientation_results_mean_4[outlier_index,occlusion_index,:],color=colors[2],label=labels[4],linestyle=linestyles[1])
error_sup=hough_orientation_results_mean_4[outlier_index,occlusion_index,:]+hough_orientation_results_std_4[outlier_index,occlusion_index,:];
error_inf=hough_orientation_results_mean_4[outlier_index,occlusion_index,:]-hough_orientation_results_std_4[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

plt.plot(error_levels,hough_orientation_results_mean_5[outlier_index,occlusion_index,:],color=colors[2],label=labels[5],linestyle=linestyles[0])
error_sup=hough_orientation_results_mean_5[outlier_index,occlusion_index,:]+hough_orientation_results_std_5[outlier_index,occlusion_index,:];
error_inf=hough_orientation_results_mean_5[outlier_index,occlusion_index,:]-hough_orientation_results_std_5[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

plt.plot(error_levels,hough_orientation_results_mean_6[outlier_index,occlusion_index,:],color=colors[3],label=labels[6],linestyle=linestyles[1])
error_sup=hough_orientation_results_mean_6[outlier_index,occlusion_index,:]+hough_orientation_results_std_6[outlier_index,occlusion_index,:];
error_inf=hough_orientation_results_mean_6[outlier_index,occlusion_index,:]-hough_orientation_results_std_6[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[6])

plt.plot(error_levels,hough_orientation_results_mean_7[outlier_index,occlusion_index,:],color=colors[3],label=labels[7],linestyle=linestyles[0])
error_sup=hough_orientation_results_mean_7[outlier_index,occlusion_index,:]+hough_orientation_results_std_7[outlier_index,occlusion_index,:];
error_inf=hough_orientation_results_mean_7[outlier_index,occlusion_index,:]-hough_orientation_results_std_7[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[3])

manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('noise standard deviation [% of cylinder radius]',fontsize=fontsize_)
plt.ylabel('absolute orientation error [$^\circ$]',fontsize=fontsize_)

plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
plt.legend(fontsize=fontsize_)
#plt.show()
plt.savefig('noise_orientation_error.pdf',format='pdf',pad_inches=0.8)

### Position
plt.figure(figsize=(8, 6))
plt.plot(error_levels,hough_position_results_mean_0[outlier_index,occlusion_index,:],color=colors[0],label=labels[0],linestyle=linestyles[1])
error_sup=hough_position_results_mean_0[outlier_index,occlusion_index,:]+hough_position_results_std_0[outlier_index,occlusion_index,:];
error_inf=hough_position_results_mean_0[outlier_index,occlusion_index,:]-hough_position_results_std_0[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(error_levels,hough_position_results_mean_1[outlier_index,occlusion_index,:],color=colors[0],label=labels[1],linestyle=linestyles[0])
error_sup=hough_position_results_mean_1[outlier_index,occlusion_index,:]+hough_position_results_std_1[outlier_index,occlusion_index,:];
error_inf=hough_position_results_mean_1[outlier_index,occlusion_index,:]-hough_position_results_std_1[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(error_levels,hough_position_results_mean_2[outlier_index,occlusion_index,:],color=colors[1],label=labels[2],linestyle=linestyles[1])
error_sup=hough_position_results_mean_2[outlier_index,occlusion_index,:]+hough_position_results_std_2[outlier_index,occlusion_index,:];
error_inf=hough_position_results_mean_2[outlier_index,occlusion_index,:]-hough_position_results_std_2[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(error_levels,hough_position_results_mean_3[outlier_index,occlusion_index,:],color=colors[1],label=labels[3],linestyle=linestyles[0])
error_sup=hough_position_results_mean_3[outlier_index,occlusion_index,:]+hough_position_results_std_3[outlier_index,occlusion_index,:];
error_inf=hough_position_results_mean_3[outlier_index,occlusion_index,:]-hough_position_results_std_3[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(error_levels,hough_position_results_mean_4[outlier_index,occlusion_index,:],color=colors[2],label=labels[4],linestyle=linestyles[1])
error_sup=hough_position_results_mean_4[outlier_index,occlusion_index,:]+hough_position_results_std_4[outlier_index,occlusion_index,:];
error_inf=hough_position_results_mean_4[outlier_index,occlusion_index,:]-hough_position_results_std_4[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

plt.plot(error_levels,hough_position_results_mean_5[outlier_index,occlusion_index,:],color=colors[2],label=labels[5],linestyle=linestyles[0])
error_sup=hough_position_results_mean_3[outlier_index,occlusion_index,:]+hough_position_results_std_5[outlier_index,occlusion_index,:];
error_inf=hough_position_results_mean_3[outlier_index,occlusion_index,:]-hough_position_results_std_5[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

plt.plot(error_levels,hough_position_results_mean_6[outlier_index,occlusion_index,:],color=colors[3],label=labels[6],linestyle=linestyles[1])
error_sup=hough_position_results_mean_6[outlier_index,occlusion_index,:]+hough_position_results_std_6[outlier_index,occlusion_index,:];
error_inf=hough_position_results_mean_6[outlier_index,occlusion_index,:]-hough_position_results_std_6[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[3])

plt.plot(error_levels,hough_position_results_mean_7[outlier_index,occlusion_index,:],color=colors[3],label=labels[7],linestyle=linestyles[0])
error_sup=hough_position_results_mean_7[outlier_index,occlusion_index,:]+hough_position_results_std_7[outlier_index,occlusion_index,:];
error_inf=hough_position_results_mean_7[outlier_index,occlusion_index,:]-hough_position_results_std_7[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[3])

manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('noise standard deviation [% of cylinder radius]',fontsize=fontsize_)
plt.ylabel('absolute position error [m]',fontsize=fontsize_)
plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
#plt.show()
plt.savefig('noise_position_error.pdf',format='pdf')

### Radius
plt.figure(figsize=(8, 6))
plt.plot(error_levels,hough_radius_results_mean_0[outlier_index,occlusion_index,:],color=colors[0],label=labels[0],linestyle=linestyles[1])
error_sup=hough_radius_results_mean_0[outlier_index,occlusion_index,:]+hough_radius_results_std_0[outlier_index,occlusion_index,:];
error_inf=hough_radius_results_mean_0[outlier_index,occlusion_index,:]-hough_radius_results_std_0[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(error_levels,hough_radius_results_mean_1[outlier_index,occlusion_index,:],color=colors[0],label=labels[1],linestyle=linestyles[0])
error_sup=hough_radius_results_mean_1[outlier_index,occlusion_index,:]+hough_radius_results_std_1[outlier_index,occlusion_index,:];
error_inf=hough_radius_results_mean_1[outlier_index,occlusion_index,:]-hough_radius_results_std_1[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(error_levels,hough_radius_results_mean_2[outlier_index,occlusion_index,:],color=colors[1],label=labels[2],linestyle=linestyles[1])
error_sup=hough_radius_results_mean_2[outlier_index,occlusion_index,:]+hough_radius_results_std_2[outlier_index,occlusion_index,:];
error_inf=hough_radius_results_mean_2[outlier_index,occlusion_index,:]-hough_radius_results_std_2[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(error_levels,hough_radius_results_mean_3[outlier_index,occlusion_index,:],color=colors[1],label=labels[3],linestyle=linestyles[0])
error_sup=hough_radius_results_mean_3[outlier_index,occlusion_index,:]+hough_radius_results_std_3[outlier_index,occlusion_index,:];
error_inf=hough_radius_results_mean_3[outlier_index,occlusion_index,:]-hough_radius_results_std_3[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(error_levels,hough_radius_results_mean_4[outlier_index,occlusion_index,:],color=colors[2],label=labels[4],linestyle=linestyles[1])
error_sup=hough_radius_results_mean_4[outlier_index,occlusion_index,:]+hough_radius_results_std_4[outlier_index,occlusion_index,:];
error_inf=hough_radius_results_mean_4[outlier_index,occlusion_index,:]-hough_radius_results_std_4[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

plt.plot(error_levels,hough_radius_results_mean_5[outlier_index,occlusion_index,:],color=colors[2],label=labels[5],linestyle=linestyles[0])
error_sup=hough_radius_results_mean_5[outlier_index,occlusion_index,:]+hough_radius_results_std_5[outlier_index,occlusion_index,:];
error_inf=hough_radius_results_mean_5[outlier_index,occlusion_index,:]-hough_radius_results_std_5[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

plt.plot(error_levels,hough_radius_results_mean_6[outlier_index,occlusion_index,:],color=colors[3],label=labels[6],linestyle=linestyles[1])
error_sup=hough_radius_results_mean_6[outlier_index,occlusion_index,:]+hough_radius_results_std_6[outlier_index,occlusion_index,:];
error_inf=hough_radius_results_mean_6[outlier_index,occlusion_index,:]-hough_radius_results_std_6[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[3])

plt.plot(error_levels,hough_radius_results_mean_7[outlier_index,occlusion_index,:],color=colors[3],label=labels[7],linestyle=linestyles[0])
error_sup=hough_radius_results_mean_7[outlier_index,occlusion_index,:]+hough_radius_results_std_7[outlier_index,occlusion_index,:];
error_inf=hough_radius_results_mean_7[outlier_index,occlusion_index,:]-hough_radius_results_std_7[outlier_index,occlusion_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[3])

manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('noise standard deviation [% of cylinder radius]',fontsize=fontsize_)
plt.ylabel('absolute radius error [m]',fontsize=fontsize_)
plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
#plt.show()
plt.savefig('noise_radius_error.pdf',format='pdf')


### Plots (outliers)

### Orientation
plt.figure(figsize=(8, 6))
print hough_orientation_results_mean_0[:,occlusion_index,noise_index]
plt.plot(outlier_levels_,hough_orientation_results_mean_0[:,occlusion_index,noise_index],color=colors[0],label=labels[0],linestyle=linestyles[1])
error_sup=hough_orientation_results_mean_0[:,occlusion_index,noise_index]+hough_orientation_results_std_0[:,occlusion_index,noise_index];
error_inf=hough_orientation_results_mean_0[:,occlusion_index,noise_index]-hough_orientation_results_std_0[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(outlier_levels_,hough_orientation_results_mean_1[:,occlusion_index,noise_index],color=colors[0],label=labels[1],linestyle=linestyles[0])
error_sup=hough_orientation_results_mean_1[:,occlusion_index,noise_index]+hough_orientation_results_std_1[:,occlusion_index,noise_index];
error_inf=hough_orientation_results_mean_1[:,occlusion_index,noise_index]-hough_orientation_results_std_1[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(outlier_levels_,hough_orientation_results_mean_2[:,occlusion_index,noise_index],color=colors[1],label=labels[2],linestyle=linestyles[1])
error_sup=hough_orientation_results_mean_2[:,occlusion_index,noise_index]+hough_orientation_results_std_2[:,occlusion_index,noise_index];
error_inf=hough_orientation_results_mean_2[:,occlusion_index,noise_index]-hough_orientation_results_std_2[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(outlier_levels_,hough_orientation_results_mean_3[:,occlusion_index,noise_index],color=colors[1],label=labels[3],linestyle=linestyles[0])
error_sup=hough_orientation_results_mean_3[:,occlusion_index,noise_index]+hough_orientation_results_std_3[:,occlusion_index,noise_index];
error_inf=hough_orientation_results_mean_3[:,occlusion_index,noise_index]-hough_orientation_results_std_3[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(outlier_levels_,hough_orientation_results_mean_4[:,occlusion_index,noise_index],color=colors[2],label=labels[4],linestyle=linestyles[1])
error_sup=hough_orientation_results_mean_4[:,occlusion_index,noise_index]+hough_orientation_results_std_4[:,occlusion_index,noise_index];
error_inf=hough_orientation_results_mean_4[:,occlusion_index,noise_index]-hough_orientation_results_std_4[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

plt.plot(outlier_levels_,hough_orientation_results_mean_5[:,occlusion_index,noise_index],color=colors[2],label=labels[5],linestyle=linestyles[0])
error_sup=hough_orientation_results_mean_5[:,occlusion_index,noise_index]+hough_orientation_results_std_5[:,occlusion_index,noise_index];
error_inf=hough_orientation_results_mean_5[:,occlusion_index,noise_index]-hough_orientation_results_std_5[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

plt.plot(outlier_levels_,hough_orientation_results_mean_6[:,occlusion_index,noise_index],color=colors[3],label=labels[6],linestyle=linestyles[1])
error_sup=hough_orientation_results_mean_6[:,occlusion_index,noise_index]+hough_orientation_results_std_6[:,occlusion_index,noise_index];
error_inf=hough_orientation_results_mean_6[:,occlusion_index,noise_index]-hough_orientation_results_std_6[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[3])

plt.plot(outlier_levels_,hough_orientation_results_mean_7[:,occlusion_index,noise_index],color=colors[3],label=labels[7],linestyle=linestyles[0])
error_sup=hough_orientation_results_mean_7[:,occlusion_index,noise_index]+hough_orientation_results_std_7[:,occlusion_index,noise_index];
error_inf=hough_orientation_results_mean_7[:,occlusion_index,noise_index]-hough_orientation_results_std_7[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[3])

manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('outliers [% of cylinder surface points]',fontsize=fontsize_)
plt.ylabel('absolute orientation error [$^\circ$]',fontsize=fontsize_)
plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
plt.legend(fontsize=fontsize_)
plt.savefig('outliers_orientation_error.pdf',format='pdf')

### Position
plt.figure(figsize=(8, 6))
plt.plot(outlier_levels_,hough_position_results_mean_0[:,occlusion_index,noise_index],color=colors[0],label=labels[0],linestyle=linestyles[1])
error_sup=hough_position_results_mean_0[:,occlusion_index,noise_index]+hough_position_results_std_0[:,occlusion_index,noise_index];
error_inf=hough_position_results_mean_0[:,occlusion_index,noise_index]-hough_position_results_std_0[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(outlier_levels_,hough_position_results_mean_1[:,occlusion_index,noise_index],color=colors[0],label=labels[1],linestyle=linestyles[0])
error_sup=hough_position_results_mean_1[:,occlusion_index,noise_index]+hough_position_results_std_1[:,occlusion_index,noise_index];
error_inf=hough_position_results_mean_1[:,occlusion_index,noise_index]-hough_position_results_std_1[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(outlier_levels_,hough_position_results_mean_2[:,occlusion_index,noise_index],color=colors[1],label=labels[2],linestyle=linestyles[1])
error_sup=hough_position_results_mean_2[:,occlusion_index,noise_index]+hough_position_results_std_2[:,occlusion_index,noise_index];
error_inf=hough_position_results_mean_2[:,occlusion_index,noise_index]-hough_position_results_std_2[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(outlier_levels_,hough_position_results_mean_3[:,occlusion_index,noise_index],color=colors[1],label=labels[3],linestyle=linestyles[0])
error_sup=hough_position_results_mean_3[:,occlusion_index,noise_index]+hough_position_results_std_3[:,occlusion_index,noise_index];
error_inf=hough_position_results_mean_3[:,occlusion_index,noise_index]-hough_position_results_std_3[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(outlier_levels_,hough_position_results_mean_4[:,occlusion_index,noise_index],color=colors[2],label=labels[4],linestyle=linestyles[1])
error_sup=hough_position_results_mean_4[:,occlusion_index,noise_index]+hough_position_results_std_4[:,occlusion_index,noise_index];
error_inf=hough_position_results_mean_4[:,occlusion_index,noise_index]-hough_position_results_std_4[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

plt.plot(outlier_levels_,hough_position_results_mean_5[:,occlusion_index,noise_index],color=colors[2],label=labels[5],linestyle=linestyles[0])
error_sup=hough_position_results_mean_5[:,occlusion_index,noise_index]+hough_position_results_std_5[:,occlusion_index,noise_index];
error_inf=hough_position_results_mean_5[:,occlusion_index,noise_index]-hough_position_results_std_5[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

plt.plot(outlier_levels_,hough_position_results_mean_6[:,occlusion_index,noise_index],color=colors[3],label=labels[6],linestyle=linestyles[1])
error_sup=hough_position_results_mean_6[:,occlusion_index,noise_index]+hough_position_results_std_6[:,occlusion_index,noise_index];
error_inf=hough_position_results_mean_6[:,occlusion_index,noise_index]-hough_position_results_std_6[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[3])

plt.plot(outlier_levels_,hough_position_results_mean_7[:,occlusion_index,noise_index],color=colors[3],label=labels[7],linestyle=linestyles[0])
error_sup=hough_position_results_mean_7[:,occlusion_index,noise_index]+hough_position_results_std_7[:,occlusion_index,noise_index];
error_inf=hough_position_results_mean_7[:,occlusion_index,noise_index]-hough_position_results_std_7[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[3])

manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('outliers [% of cylinder surface points]',fontsize=fontsize_)
plt.ylabel('absolute position error [m]',fontsize=fontsize_)
plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
plt.legend(fontsize=fontsize_)
plt.savefig('outliers_position_error.pdf',format='pdf')


### Radius
plt.figure(figsize=(8, 6))
plt.plot(outlier_levels_,hough_radius_results_mean_0[:,occlusion_index,noise_index],color=colors[0],label=labels[0],linestyle=linestyles[1])
error_sup=hough_radius_results_mean_0[:,occlusion_index,noise_index]+hough_radius_results_std_0[:,occlusion_index,noise_index];
error_inf=hough_radius_results_mean_0[:,occlusion_index,noise_index]-hough_radius_results_std_0[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(outlier_levels_,hough_radius_results_mean_1[:,occlusion_index,noise_index],color=colors[0],label=labels[1],linestyle=linestyles[0])
error_sup=hough_radius_results_mean_1[:,occlusion_index,noise_index]+hough_radius_results_std_1[:,occlusion_index,noise_index];
error_inf=hough_radius_results_mean_1[:,occlusion_index,noise_index]-hough_radius_results_std_1[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(outlier_levels_,hough_radius_results_mean_2[:,occlusion_index,noise_index],color=colors[1],label=labels[2],linestyle=linestyles[1])
error_sup=hough_radius_results_mean_2[:,occlusion_index,noise_index]+hough_radius_results_std_2[:,occlusion_index,noise_index];
error_inf=hough_radius_results_mean_2[:,occlusion_index,noise_index]-hough_radius_results_std_2[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(outlier_levels_,hough_radius_results_mean_3[:,occlusion_index,noise_index],color=colors[1],label=labels[3],linestyle=linestyles[0])
error_sup=hough_radius_results_mean_3[:,occlusion_index,noise_index]+hough_radius_results_std_3[:,occlusion_index,noise_index];
error_inf=hough_radius_results_mean_3[:,occlusion_index,noise_index]-hough_radius_results_std_3[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(outlier_levels_,hough_radius_results_mean_4[:,occlusion_index,noise_index],color=colors[2],label=labels[4],linestyle=linestyles[1])
error_sup=hough_radius_results_mean_4[:,occlusion_index,noise_index]+hough_radius_results_std_4[:,occlusion_index,noise_index];
error_inf=hough_radius_results_mean_4[:,occlusion_index,noise_index]-hough_radius_results_std_4[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

plt.plot(outlier_levels_,hough_radius_results_mean_5[:,occlusion_index,noise_index],color=colors[2],label=labels[5],linestyle=linestyles[0])
error_sup=hough_radius_results_mean_5[:,occlusion_index,noise_index]+hough_radius_results_std_5[:,occlusion_index,noise_index];
error_inf=hough_radius_results_mean_5[:,occlusion_index,noise_index]-hough_radius_results_std_5[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

plt.plot(outlier_levels_,hough_radius_results_mean_6[:,occlusion_index,noise_index],color=colors[3],label=labels[6],linestyle=linestyles[1])
error_sup=hough_radius_results_mean_6[:,occlusion_index,noise_index]+hough_radius_results_std_6[:,occlusion_index,noise_index];
error_inf=hough_radius_results_mean_6[:,occlusion_index,noise_index]-hough_radius_results_std_6[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[3])

plt.plot(outlier_levels_,hough_radius_results_mean_7[:,occlusion_index,noise_index],color=colors[3],label=labels[7],linestyle=linestyles[0])
error_sup=hough_radius_results_mean_7[:,occlusion_index,noise_index]+hough_radius_results_std_7[:,occlusion_index,noise_index];
error_inf=hough_radius_results_mean_7[:,occlusion_index,noise_index]-hough_radius_results_std_7[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[3])

manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('outliers [% of cylinder surface points]',fontsize=fontsize_)
plt.ylabel('absolute radius error [m]',fontsize=fontsize_)

plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
plt.legend(fontsize=fontsize_)
plt.savefig('outliers_radius_error.pdf',format='pdf')



### Plots (occlusion)

### Orientation
plt.figure(figsize=(8, 6))
plt.plot(occlusion_levels_,hough_orientation_results_mean_0[outlier_index,:,noise_index],color=colors[0],label=labels[0])
error_sup=hough_orientation_results_mean_0[outlier_index,:,noise_index]+hough_orientation_results_std_0[outlier_index,:,noise_index];
error_inf=hough_orientation_results_mean_0[outlier_index,:,noise_index]-hough_orientation_results_std_0[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(occlusion_levels_,hough_orientation_results_mean_1[outlier_index,:,noise_index],color=colors[0],label=labels[1])
error_sup=hough_orientation_results_mean_1[outlier_index,:,noise_index]+hough_orientation_results_std_1[outlier_index,:,noise_index];
error_inf=hough_orientation_results_mean_1[outlier_index,:,noise_index]-hough_orientation_results_std_1[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(occlusion_levels_,hough_orientation_results_mean_2[outlier_index,:,noise_index],color=colors[1],label=labels[2])
error_sup=hough_orientation_results_mean_2[outlier_index,:,noise_index]+hough_orientation_results_std_2[outlier_index,:,noise_index];
error_inf=hough_orientation_results_mean_2[outlier_index,:,noise_index]-hough_orientation_results_std_2[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(occlusion_levels_,hough_orientation_results_mean_3[outlier_index,:,noise_index],color=colors[1],label=labels[3])
error_sup=hough_orientation_results_mean_3[outlier_index,:,noise_index]+hough_orientation_results_std_3[outlier_index,:,noise_index];
error_inf=hough_orientation_results_mean_3[outlier_index,:,noise_index]-hough_orientation_results_std_3[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(occlusion_levels_,hough_orientation_results_mean_4[outlier_index,:,noise_index],color=colors[2],label=labels[4])
error_sup=hough_orientation_results_mean_4[outlier_index,:,noise_index]+hough_orientation_results_std_4[outlier_index,:,noise_index];
error_inf=hough_orientation_results_mean_4[outlier_index,:,noise_index]-hough_orientation_results_std_4[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

plt.plot(occlusion_levels_,hough_orientation_results_mean_5[outlier_index,:,noise_index],color=colors[2],label=labels[5])
error_sup=hough_orientation_results_mean_5[outlier_index,:,noise_index]+hough_orientation_results_std_5[outlier_index,:,noise_index];
error_inf=hough_orientation_results_mean_5[outlier_index,:,noise_index]-hough_orientation_results_std_5[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

plt.plot(occlusion_levels_,hough_orientation_results_mean_6[outlier_index,:,noise_index],color=colors[3],label=labels[6])
error_sup=hough_orientation_results_mean_6[outlier_index,:,noise_index]+hough_orientation_results_std_6[outlier_index,:,noise_index];
error_inf=hough_orientation_results_mean_6[outlier_index,:,noise_index]-hough_orientation_results_std_6[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[3])

plt.plot(occlusion_levels_,hough_orientation_results_mean_7[outlier_index,:,noise_index],color=colors[3],label=labels[7])
error_sup=hough_orientation_results_mean_7[outlier_index,:,noise_index]+hough_orientation_results_std_7[outlier_index,:,noise_index];
error_inf=hough_orientation_results_mean_7[outlier_index,:,noise_index]-hough_orientation_results_std_7[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[3])

manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('outliers [% of cylinder surface points]',fontsize=fontsize_)
plt.ylabel('absolute orientation error [$^\circ$]',fontsize=fontsize_)
plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
plt.legend(fontsize=fontsize_)
plt.savefig('occlusion_orientation_error.pdf',format='pdf')

### Position
plt.figure(figsize=(8, 6))
plt.plot(occlusion_levels_,hough_position_results_mean_0[outlier_index,:,noise_index],color=colors[0],label=labels[0])
error_sup=hough_position_results_mean_0[outlier_index,:,noise_index]+hough_position_results_std_0[outlier_index,:,noise_index];
error_inf=hough_position_results_mean_0[outlier_index,:,noise_index]-hough_position_results_std_0[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(occlusion_levels_,hough_position_results_mean_1[outlier_index,:,noise_index],color=colors[1],label=labels[1])
error_sup=hough_position_results_mean_1[outlier_index,:,noise_index]+hough_position_results_std_1[outlier_index,:,noise_index];
error_inf=hough_position_results_mean_1[outlier_index,:,noise_index]-hough_position_results_std_1[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(occlusion_levels_,hough_position_results_mean_2[outlier_index,:,noise_index],color=colors[2],label=labels[2])
error_sup=hough_position_results_mean_2[outlier_index,:,noise_index]+hough_position_results_std_2[outlier_index,:,noise_index];
error_inf=hough_position_results_mean_2[outlier_index,:,noise_index]-hough_position_results_std_2[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

plt.plot(occlusion_levels_,hough_position_results_mean_3[outlier_index,:,noise_index],color=colors[3],label=labels[3])
error_sup=hough_position_results_mean_3[outlier_index,:,noise_index]+hough_position_results_std_3[outlier_index,:,noise_index];
error_inf=hough_position_results_mean_3[outlier_index,:,noise_index]-hough_position_results_std_3[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[3])

manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('outliers [% of cylinder surface points]',fontsize=fontsize_)
plt.ylabel('absolute position error [m]',fontsize=fontsize_)
plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
plt.legend(fontsize=fontsize_)
plt.savefig('occlusion_position_error.pdf',format='pdf')


### Radius
plt.figure(figsize=(8, 6))
plt.plot(occlusion_levels_,hough_radius_results_mean_0[outlier_index,:,noise_index],color=colors[0],label=labels[0])
error_sup=hough_radius_results_mean_0[outlier_index,:,noise_index]+hough_radius_results_std_0[outlier_index,:,noise_index];
error_inf=hough_radius_results_mean_0[outlier_index,:,noise_index]-hough_radius_results_std_0[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(occlusion_levels_,hough_radius_results_mean_1[outlier_index,:,noise_index],color=colors[1],label=labels[1])
error_sup=hough_radius_results_mean_1[outlier_index,:,noise_index]+hough_radius_results_std_1[outlier_index,:,noise_index];
error_inf=hough_radius_results_mean_1[outlier_index,:,noise_index]-hough_radius_results_std_1[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(occlusion_levels_,hough_radius_results_mean_2[outlier_index,:,noise_index],color=colors[2],label=labels[2])
error_sup=hough_radius_results_mean_2[outlier_index,:,noise_index]+hough_radius_results_std_2[outlier_index,:,noise_index];
error_inf=hough_radius_results_mean_2[outlier_index,:,noise_index]-hough_radius_results_std_2[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

plt.plot(occlusion_levels_,hough_radius_results_mean_3[outlier_index,:,noise_index],color=colors[3],label=labels[3])
error_sup=hough_radius_results_mean_3[outlier_index,:,noise_index]+hough_radius_results_std_3[outlier_index,:,noise_index];
error_inf=hough_radius_results_mean_3[outlier_index,:,noise_index]-hough_radius_results_std_3[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[3])

manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('outliers [% of cylinder surface points]',fontsize=fontsize_)
plt.ylabel('absolute radius error [m]',fontsize=fontsize_)

plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
plt.legend(fontsize=fontsize_)
plt.savefig('occlusion_radius_error.pdf',format='pdf')
plt.show()

