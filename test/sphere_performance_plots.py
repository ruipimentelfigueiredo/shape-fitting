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
home=path.expanduser('~/ws/src/shape_detection_fitting/lib/')

def to_percent(y, position):
    # Ignore the passed in position. This has the effect of scaling the default
    # tick locations.
    s = str(100 * y)

    # The percent symbol needs escaping in latex
    if plt.rcParams['text.usetex'] is True:
        return s #+ r'$\%$'
    else:
        return s #+ '%'

radii=1
iterations=500
ground_truth_size=radii
outlier_levels=1
noise_levels_number=11
noise_index=0
outlier_index=0
occlusion_index=0
alpha_=0.1
fontsize_=20
error_levels=[0,5,10,15,20,25,30,35,40,45,50]
noise_levels_number=len(error_levels)
#outlier_levels_=[0,25,50,75,100,125,150,175,200]
outlier_levels_=[0.0]
occlusion_levels_=[0.0]
occlusion_levels_number=1

colors=['green','blue','red','black']
labels=['Ours (Unbiased)','Ours (Unbiased and soft-voting)','Ours (Weak Vertical-Bias)','Ours (Weak Vertical-Bias and soft-voting)','Ours (Strong Vertical-Bias)','Ours (Strong Vertical-Bias and soft-voting)']
linestyles = ['-', '--']
linethickness=[1, 2, 3, 4, 5]

#POSITION
hough_position_results_0=[]
hough_position_results_1=[]
hough_position_results_2=[]
hough_position_results_3=[]

hough_position_file_0 = open(home + "shape-fitting/dataset/sphere/results/position_noise_0.txt", "r") 
hough_position_file_1 = open(home + "shape-fitting/dataset/sphere/results/position_noise_1.txt", "r") 

for line in hough_position_file_0:
  hough_position_results_0.append(float(line))
hough_position_results_0 = np.array(hough_position_results_0).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_position_file_1:
  hough_position_results_1.append(float(line))
hough_position_results_1 = np.array(hough_position_results_1).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)


#RADIUS
hough_radius_results_0=[]
hough_radius_results_1=[]
hough_radius_results_2=[]
hough_radius_results_3=[]

hough_radius_file_0 = open(home + "shape-fitting/dataset/sphere/results/radius_noise_0.txt", "r") 
hough_radius_file_1 = open(home + "shape-fitting/dataset/sphere/results/radius_noise_1.txt", "r") 

for line in hough_radius_file_0:
  hough_radius_results_0.append(float(line))
hough_radius_results_0 = np.array(hough_radius_results_0).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)

for line in hough_radius_file_1:
  hough_radius_results_1.append(float(line))
hough_radius_results_1 = np.array(hough_radius_results_1).reshape(iterations,outlier_levels,occlusion_levels_number,ground_truth_size,noise_levels_number)


# compute position average and standard deviation
hough_position_results_mean_0 = np.mean(hough_position_results_0, axis=(0,3))
hough_position_results_std_0  = np.std(hough_position_results_0, axis=(0,3))

hough_position_results_mean_1 = np.mean(hough_position_results_1, axis=(0,3))
hough_position_results_std_1  = np.std(hough_position_results_1, axis=(0,3))

# compute radius average and standard deviation
hough_radius_results_mean_0 = np.mean(hough_radius_results_0, axis=(0,3))
hough_radius_results_std_0  = np.std(hough_radius_results_0, axis=(0,3))

hough_radius_results_mean_1 = np.mean(hough_radius_results_1, axis=(0,3))
hough_radius_results_std_1  = np.std(hough_radius_results_1, axis=(0,3))


### Plots (noise)

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

manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('noise standard deviation [% of sphere radius]',fontsize=fontsize_)
plt.ylabel('absolute position error [m]',fontsize=fontsize_)
plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
#plt.show()
plt.legend(fontsize=fontsize_)
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

manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('noise standard deviation [% of sphere radius]',fontsize=fontsize_)
plt.ylabel('absolute radius error [m]',fontsize=fontsize_)
plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
#plt.show()
plt.savefig('noise_radius_error.pdf',format='pdf')


### Plots (outliers)

### Position
plt.figure(figsize=(8, 6))
plt.plot(outlier_levels_,hough_position_results_mean_0[:,occlusion_index,noise_index],color=colors[0],label=labels[0],linestyle=linestyles[1])
error_sup=hough_position_results_mean_0[:,occlusion_index,noise_index]+hough_position_results_std_0[:,occlusion_index,noise_index];
error_inf=hough_position_results_mean_0[:,occlusion_index,noise_index]-hough_position_results_std_0[:,occlusion_index,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(error_levels,hough_radius_results_mean_1[:,occlusion_index,noise_index],color=colors[0],label=labels[1],linestyle=linestyles[0])
error_sup=hough_radius_results_mean_1[:,occlusion_index,noise_index]+hough_radius_results_std_1[:,occlusion_index,noise_index];
error_inf=hough_radius_results_mean_1[:,occlusion_index,noise_index]-hough_radius_results_std_1[:,occlusion_index,noise_index];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('outliers [% of sphere surface points]',fontsize=fontsize_)
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


manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('outliers [% of sphere surface points]',fontsize=fontsize_)
plt.ylabel('absolute radius error [m]',fontsize=fontsize_)

plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
plt.legend(fontsize=fontsize_)
plt.savefig('outliers_radius_error.pdf',format='pdf')



### Plots (occlusion)

### Position
plt.figure(figsize=(8, 6))
plt.plot(occlusion_levels_,hough_position_results_mean_0[outlier_index,:,noise_index],color=colors[0],label=labels[0])
error_sup=hough_position_results_mean_0[outlier_index,:,noise_index]+hough_position_results_std_0[outlier_index,:,noise_index];
error_inf=hough_position_results_mean_0[outlier_index,:,noise_index]-hough_position_results_std_0[outlier_index,:,noise_index];
plt.fill_between(occlusion_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])


manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('outliers [% of sphere surface points]',fontsize=fontsize_)
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


manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('outliers [% of sphere surface points]',fontsize=fontsize_)
plt.ylabel('absolute radius error [m]',fontsize=fontsize_)

plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
plt.legend(fontsize=fontsize_)
plt.savefig('occlusion_radius_error.pdf',format='pdf')
plt.show()

