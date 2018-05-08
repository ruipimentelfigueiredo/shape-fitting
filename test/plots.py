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

heights=1
radii=1
iterations=200
ground_truth_size=heights*radii*iterations
outlier_levels=5
noise_levels_number=11
noise_index=4
outlier_index=0
alpha_=0.4
fontsize_=20
error_levels=[0.1,5,10,15,20,25,30,35,40,45,50]
outlier_levels_=[0,50,100,150,200]
colors=['black','blue','red','gray']
labels=['Rabbani et al.','Ours Unbiased','Ours Biased']

#ORIENTATION
hough_orientation_results_0=[]
hough_orientation_results_1=[]
hough_orientation_results_2=[]

hough_orientation_file_0 = open(home + "shape-fitting/dataset/results/orientation_noise_0.txt", "r") 
hough_orientation_file_1 = open(home + "shape-fitting/dataset/results/orientation_noise_1.txt", "r") 
hough_orientation_file_2 = open(home + "shape-fitting/dataset/results/orientation_noise_2.txt", "r") 


for line in hough_orientation_file_0:
  hough_orientation_results_0.append(180.0*float(line)/math.pi)
hough_orientation_results_0 = np.array(hough_orientation_results_0).reshape(ground_truth_size,outlier_levels,noise_levels_number)

for line in hough_orientation_file_1:
  hough_orientation_results_1.append(180.0*float(line)/math.pi)
hough_orientation_results_1 = np.array(hough_orientation_results_1).reshape(ground_truth_size,outlier_levels,noise_levels_number)

for line in hough_orientation_file_2:
  hough_orientation_results_2.append(180.0*float(line)/math.pi)
hough_orientation_results_2 = np.array(hough_orientation_results_2).reshape(ground_truth_size,outlier_levels,noise_levels_number)

#POSITION
hough_position_results_0=[]
hough_position_results_1=[]
hough_position_results_2=[]

hough_position_file_0 = open(home + "shape-fitting/dataset/results/position_noise_0.txt", "r") 
hough_position_file_1 = open(home + "shape-fitting/dataset/results/position_noise_1.txt", "r") 
hough_position_file_2 = open(home + "shape-fitting/dataset/results/position_noise_2.txt", "r") 

for line in hough_position_file_0:
  hough_position_results_0.append(float(line))
hough_position_results_0 = np.array(hough_position_results_0).reshape(ground_truth_size,outlier_levels,noise_levels_number)

for line in hough_position_file_1:
  hough_position_results_1.append(float(line))
hough_position_results_1 = np.array(hough_position_results_1).reshape(ground_truth_size,outlier_levels,noise_levels_number)

for line in hough_position_file_2:
  hough_position_results_2.append(float(line))
hough_position_results_2 = np.array(hough_position_results_2).reshape(ground_truth_size,outlier_levels,noise_levels_number)

#RADIUS
hough_radius_results_0=[]
hough_radius_results_1=[]
hough_radius_results_2=[]

hough_radius_file_0 = open(home + "shape-fitting/dataset/results/radius_noise_0.txt", "r") 
hough_radius_file_1 = open(home + "shape-fitting/dataset/results/radius_noise_1.txt", "r") 
hough_radius_file_2 = open(home + "shape-fitting/dataset/results/radius_noise_2.txt", "r") 

for line in hough_radius_file_0:
  hough_radius_results_0.append(float(line))
hough_radius_results_0 = np.array(hough_radius_results_0).reshape(ground_truth_size,outlier_levels,noise_levels_number)

for line in hough_radius_file_1:
  hough_radius_results_1.append(float(line))
hough_radius_results_1 = np.array(hough_radius_results_1).reshape(ground_truth_size,outlier_levels,noise_levels_number)

for line in hough_radius_file_2:
  hough_radius_results_2.append(float(line))
hough_radius_results_2 = np.array(hough_radius_results_2).reshape(ground_truth_size,outlier_levels,noise_levels_number)





# compute orientation average and standard deviation
hough_orientation_results_mean_0 = np.mean(hough_orientation_results_0, axis=0)
hough_orientation_results_std_0  = np.std(hough_orientation_results_0, axis=0)

hough_orientation_results_mean_1 = np.mean(hough_orientation_results_1, axis=0)
hough_orientation_results_std_1  = np.std(hough_orientation_results_1, axis=0)

hough_orientation_results_mean_2 = np.mean(hough_orientation_results_2, axis=0)
hough_orientation_results_std_2  = np.std(hough_orientation_results_2, axis=0)

# compute position average and standard deviation
hough_position_results_mean_0 = np.mean(hough_position_results_0, axis=0)
hough_position_results_std_0  = np.std(hough_position_results_0, axis=0)

hough_position_results_mean_1 = np.mean(hough_position_results_1, axis=0)
hough_position_results_std_1  = np.std(hough_position_results_1, axis=0)

hough_position_results_mean_2 = np.mean(hough_position_results_2, axis=0)
hough_position_results_std_2  = np.std(hough_position_results_2, axis=0)

# compute radius average and standard deviation
hough_radius_results_mean_0 = np.mean(hough_radius_results_0, axis=0)
hough_radius_results_std_0  = np.std(hough_radius_results_0, axis=0)

hough_radius_results_mean_1 = np.mean(hough_radius_results_1, axis=0)
hough_radius_results_std_1  = np.std(hough_radius_results_1, axis=0)

hough_radius_results_mean_2 = np.mean(hough_radius_results_2, axis=0)
hough_radius_results_std_2  = np.std(hough_radius_results_2, axis=0)


### Plots (noise)

### Orientation
plt.figure(figsize=(8, 8))
plt.plot(error_levels,hough_orientation_results_mean_0[outlier_index,:],color=colors[0],label=labels[0])
error_sup=hough_orientation_results_mean_0[outlier_index,:]+hough_orientation_results_std_0[outlier_index,:];
error_inf=hough_orientation_results_mean_0[outlier_index,:]-hough_orientation_results_std_0[outlier_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(error_levels,hough_orientation_results_mean_1[outlier_index,:],color=colors[1],label=labels[1])
error_sup=hough_orientation_results_mean_1[outlier_index,:]+hough_orientation_results_std_1[outlier_index,:];
error_inf=hough_orientation_results_mean_1[outlier_index,:]-hough_orientation_results_std_1[outlier_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(error_levels,hough_orientation_results_mean_2[outlier_index,:],color=colors[2],label=labels[2])
error_sup=hough_orientation_results_mean_2[outlier_index,:]+hough_orientation_results_std_2[outlier_index,:];
error_inf=hough_orientation_results_mean_2[outlier_index,:]-hough_orientation_results_std_2[outlier_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])
manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('noise standard deviation [% of cylinder radius]',fontsize=fontsize_)
plt.ylabel('absolute orientation error [$^\circ$]',fontsize=fontsize_)

plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
plt.legend(fontsize=fontsize_)
#plt.show()


### Position
plt.figure()
plt.plot(error_levels,hough_position_results_mean_0[outlier_index,:],color=colors[0],label=labels[0])
error_sup=hough_position_results_mean_0[outlier_index,:]+hough_position_results_std_0[outlier_index,:];
error_inf=hough_position_results_mean_0[outlier_index,:]-hough_position_results_std_0[outlier_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(error_levels,hough_position_results_mean_1[outlier_index,:],color=colors[1],label=labels[1])
error_sup=hough_position_results_mean_1[outlier_index,:]+hough_position_results_std_1[outlier_index,:];
error_inf=hough_position_results_mean_1[outlier_index,:]-hough_position_results_std_1[outlier_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(error_levels,hough_position_results_mean_2[outlier_index,:],color=colors[2],label=labels[2])
error_sup=hough_position_results_mean_2[outlier_index,:]+hough_position_results_std_2[outlier_index,:];
error_inf=hough_position_results_mean_2[outlier_index,:]-hough_position_results_std_2[outlier_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])
manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('noise standard deviation [% of cylinder radius]',fontsize=fontsize_)
plt.ylabel('absolute position error [m]',fontsize=fontsize_)
plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
#plt.show()


### Radius
plt.figure()
plt.plot(error_levels,hough_radius_results_mean_0[outlier_index,:],color=colors[0],label=labels[0])
error_sup=hough_radius_results_mean_0[outlier_index,:]+hough_radius_results_std_0[outlier_index,:];
error_inf=hough_radius_results_mean_0[outlier_index,:]-hough_radius_results_std_0[outlier_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(error_levels,hough_radius_results_mean_1[outlier_index,:],color=colors[1],label=labels[1])
error_sup=hough_radius_results_mean_1[outlier_index,:]+hough_radius_results_std_1[outlier_index,:];
error_inf=hough_radius_results_mean_1[outlier_index,:]-hough_radius_results_std_1[outlier_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(error_levels,hough_radius_results_mean_2[outlier_index,:],color=colors[2],label=labels[2])
error_sup=hough_radius_results_mean_2[outlier_index,:]+hough_radius_results_std_2[outlier_index,:];
error_inf=hough_radius_results_mean_2[outlier_index,:]-hough_radius_results_std_2[outlier_index,:];
plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])

manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('noise standard deviation [% of cylinder radius]',fontsize=fontsize_)
plt.ylabel('absolute radius error [m]',fontsize=fontsize_)
plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
#plt.show()


### Plots (outliers)

### Orientation
plt.figure()
plt.plot(outlier_levels_,hough_orientation_results_mean_0[:,noise_index],color=colors[0],label=labels[0])
error_sup=hough_orientation_results_mean_0[:,noise_index]+hough_orientation_results_std_0[:,noise_index];
error_inf=hough_orientation_results_mean_0[:,noise_index]-hough_orientation_results_std_0[:,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(outlier_levels_,hough_orientation_results_mean_1[:,noise_index],color=colors[1],label=labels[1])
error_sup=hough_orientation_results_mean_1[:,noise_index]+hough_orientation_results_std_1[:,noise_index];
error_inf=hough_orientation_results_mean_1[:,noise_index]-hough_orientation_results_std_1[:,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(outlier_levels_,hough_orientation_results_mean_2[:,noise_index],color=colors[2],label=labels[2])
error_sup=hough_orientation_results_mean_2[:,noise_index]+hough_orientation_results_std_2[:,noise_index];
error_inf=hough_orientation_results_mean_2[:,noise_index]-hough_orientation_results_std_2[:,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])
manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('outliers [% of cylinder surface points]',fontsize=fontsize_)
plt.ylabel('absolute orientation error [$^\circ$]',fontsize=fontsize_)

plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
plt.legend(fontsize=fontsize_)
#plt.show()

### Position
plt.figure()
plt.plot(outlier_levels_,hough_position_results_mean_0[:,noise_index],color=colors[0],label=labels[0])
error_sup=hough_position_results_mean_0[:,noise_index]+hough_position_results_std_0[:,noise_index];
error_inf=hough_position_results_mean_0[:,noise_index]-hough_position_results_std_0[:,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(outlier_levels_,hough_position_results_mean_1[:,noise_index],color=colors[1],label=labels[1])
error_sup=hough_position_results_mean_1[:,noise_index]+hough_position_results_std_1[:,noise_index];
error_inf=hough_position_results_mean_1[:,noise_index]-hough_position_results_std_1[:,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(outlier_levels_,hough_position_results_mean_2[:,noise_index],color=colors[2],label=labels[2])
error_sup=hough_position_results_mean_2[:,noise_index]+hough_position_results_std_2[:,noise_index];
error_inf=hough_position_results_mean_2[:,noise_index]-hough_position_results_std_2[:,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])
manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('outliers [% of cylinder surface points]',fontsize=fontsize_)
plt.ylabel('absolute position error [m]',fontsize=fontsize_)

plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
plt.legend(fontsize=fontsize_)

### Radius
plt.figure()
plt.plot(outlier_levels_,hough_radius_results_mean_0[:,noise_index],color=colors[0],label=labels[0])
error_sup=hough_radius_results_mean_0[:,noise_index]+hough_radius_results_std_0[:,noise_index];
error_inf=hough_radius_results_mean_0[:,noise_index]-hough_radius_results_std_0[:,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[0])

plt.plot(outlier_levels_,hough_radius_results_mean_1[:,noise_index],color=colors[1],label=labels[1])
error_sup=hough_radius_results_mean_1[:,noise_index]+hough_radius_results_std_1[:,noise_index];
error_inf=hough_radius_results_mean_1[:,noise_index]-hough_radius_results_std_1[:,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[1])

plt.plot(outlier_levels_,hough_radius_results_mean_2[:,noise_index],color=colors[2],label=labels[2])
error_sup=hough_radius_results_mean_2[:,noise_index]+hough_radius_results_std_2[:,noise_index];
error_inf=hough_radius_results_mean_2[:,noise_index]-hough_radius_results_std_2[:,noise_index];
plt.fill_between(outlier_levels_,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[2])
manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())
plt.xlabel('outliers [% of cylinder surface points]',fontsize=fontsize_)
plt.ylabel('absolute radius error [m]',fontsize=fontsize_)

plt.xticks(color='k', size=fontsize_)
plt.yticks(color='k', size=fontsize_)
plt.legend(fontsize=fontsize_)
plt.show()

