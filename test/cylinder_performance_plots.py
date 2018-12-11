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
import os 

import numpy as np
import math

#test_type='robustness_outliers'
test_type='robustness_noise'
#test_type='robustness_occlusion'

current_file = os.path.abspath(os.path.dirname(__file__)+"/..")
print(current_file)
home = os.path.join(current_file, 'dataset')

def to_percent(y, position):
    # Ignore the passed in position. This has the effect of scaling the default tick locations.
    s = str(100 * y)

    # The percent symbol needs escaping in latex
    if plt.rcParams['text.usetex'] is True:
        return s #+ r'$\%$'
    else:
        return s #+ '%'

total_algorithms=4
heights=5
radii=1
iterations=1000
ground_truth_size=heights*radii

if test_type=='robustness_noise':
    dataset_path=path.expanduser(home+'/cylinder_noise/results')
    error_levels=[0.01,0.10,0.20,0.30,0.40,0.50,0.60,0.70,0.80,0.90,1.00]
    outlier_levels=[0]
    occlusion_levels=[0]
elif test_type=='robustness_outliers':
    dataset_path=path.expanduser(home+'/cylinder_outliers/results')
    error_levels=[0]
    outlier_levels=[0.00,0.25,0.5,0.75,1.0,1.25,1.5,1.75,2.0,2.25,2.5,2.75,3.0,3.25,3.5]
    occlusion_levels=[0]
elif test_type=='robustness_occlusion':
    dataset_path=path.expanduser(home+'/cylinder_occlusion/results')
    error_levels=[0]
    outlier_levels=[0]
    occlusion_levels=[0.00,0.10,0.20,0.30,0.40,0.50,0.60,0.70,0.80,0.90]
    
occlusion_x_ticks=(0.00,0.20,0.40,0.60,0.80)
outliers_x_ticks=(0.00,0.5,1.0,1.5,2.0,2.5,3.0,3.5)
error_levels_x_ticks=(0.00,0.20,0.40,0.60,0.80,1.00)

orientation_y_ticks=(0,20,40,60,80,100)

outlier_levels_number  =len(outlier_levels)
noise_levels_number    =len(error_levels)
occlusion_levels_number=len(occlusion_levels)

noise_index=0
outlier_index=0
occlusion_index=0

alpha_=0.1
fontsize_=20

colors=['black','green','blue','orange','red','blue','aqua','red','orange','gray','brown','brown','aqua','aqua','yellow','yellow','purple','purple']
labels=['Rabbani et al.','Ours (Unbiased)','Ours (Weak Vertical-Bias)','Ours (Strong Vertical-Bias)']

linestyles=['-','--',':','-.','-','-','-','-','-','-']
linethickness=[1, 2, 3, 4, 5]
algorithms=[0,1,2,3]

hough_orientation_results=[]
hough_position_results=[]
hough_radius_results=[]
for a in range(0,total_algorithms):
	#ORIENTATION
	hough_orientation_file=open(dataset_path + "/orientation_" + str(a) + ".txt", "r")
	for line in hough_orientation_file:
  		hough_orientation_results.append(180.0*float(line)/math.pi)
	#POSITION
	hough_position_file=open(dataset_path + "/position_" + str(a) + ".txt", "r")
	for line in hough_position_file:
  		hough_position_results.append(float(line))
	#RADIUS
	hough_radius_file=open(dataset_path + "/radius_" + str(a) + ".txt", "r")
	for line in hough_radius_file:
  		hough_radius_results.append(float(line))

hough_orientation_results = np.array(hough_orientation_results).reshape(total_algorithms,iterations,outlier_levels_number,occlusion_levels_number,ground_truth_size,noise_levels_number)
hough_position_results    = np.array(hough_position_results).reshape(total_algorithms,iterations,outlier_levels_number,occlusion_levels_number,ground_truth_size,noise_levels_number)
hough_radius_results      = np.array(hough_radius_results).reshape(total_algorithms,iterations,outlier_levels_number,occlusion_levels_number,ground_truth_size,noise_levels_number)

# compute orientation average and standard deviation
hough_orientation_results_mean = np.mean(hough_orientation_results, axis=(1,4))
hough_orientation_results_std  = np.std(hough_orientation_results, axis=(1,4))

# compute position average and standard deviation
hough_position_results_mean = np.mean(hough_position_results, axis=(1,4))
hough_position_results_std  = np.std(hough_position_results, axis=(1,4))

# compute radius average and standard deviation
hough_radius_results_mean = np.mean(hough_radius_results, axis=(1,4))
hough_radius_results_std  = np.std(hough_radius_results, axis=(1,4))

### Plots (noise)
if noise_levels_number>1:
	### Orientation
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(error_levels,hough_orientation_results_mean[a,outlier_index,occlusion_index,:],color=colors[a],label=labels[a],linestyle=linestyles[a],linewidth=linethickness[a])
		error_sup=hough_orientation_results_mean[a,outlier_index,occlusion_index,:]+hough_orientation_results_std[a,outlier_index,occlusion_index,:];
		error_inf=hough_orientation_results_mean[a,outlier_index,occlusion_index,:]-hough_orientation_results_std[a,outlier_index,occlusion_index,:];
		plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('noise standard deviation [% of cylinder radius]',fontsize=fontsize_)
	plt.ylabel('absolute orientation error [$^\circ$]',fontsize=fontsize_)
	plt.xticks(error_levels_x_ticks,color='k',size=fontsize_)
	plt.yticks(orientation_y_ticks,color='k',size=fontsize_)
	plt.legend(fontsize=fontsize_)
	plt.xlim(left=min(error_levels))
	plt.xlim(right=max(error_levels))
	plt.ylim(bottom=0)
	plt.ylim(top=100)
	plt.savefig('noise_orientation_error.pdf',format='pdf',pad_inches=0.8)

	### Position
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(error_levels,hough_position_results_mean[a,outlier_index,occlusion_index,:],color=colors[a],label=labels[a],linestyle=linestyles[a],linewidth=linethickness[a])
		error_sup=hough_position_results_mean[a,outlier_index,occlusion_index,:]+hough_position_results_std[a,outlier_index,occlusion_index,:];
		error_inf=hough_position_results_mean[a,outlier_index,occlusion_index,:]-hough_position_results_std[a,outlier_index,occlusion_index,:];
		plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('noise standard deviation [% of cylinder radius]',fontsize=fontsize_)
	plt.ylabel('absolute position error [m]',fontsize=fontsize_)
	plt.xticks(error_levels_x_ticks, color='k', size=fontsize_)
	plt.yticks(color='k',size=fontsize_)
	plt.xlim(left=min(error_levels))
	plt.xlim(right=max(error_levels))
	plt.ylim(bottom=0)
	plt.savefig('noise_position_error.pdf',format='pdf')

	### Radius
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(error_levels,hough_radius_results_mean[a,outlier_index,occlusion_index,:],color=colors[a],label=labels[a],linestyle=linestyles[a],linewidth=linethickness[a])
		error_sup=hough_radius_results_mean[a,outlier_index,occlusion_index,:]+hough_radius_results_std[a,outlier_index,occlusion_index,:];
		error_inf=hough_radius_results_mean[a,outlier_index,occlusion_index,:]-hough_radius_results_std[a,outlier_index,occlusion_index,:];
		plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('noise standard deviation [% of cylinder radius]',fontsize=fontsize_)
	plt.ylabel('absolute radius error [m]',fontsize=fontsize_)
	plt.xticks(error_levels_x_ticks,color='k',size=fontsize_)
	plt.yticks(color='k',size=fontsize_)
	plt.xlim(left=min(error_levels))
	plt.xlim(right=max(error_levels))
	plt.ylim(bottom=0.00)
	plt.ylim(top=0.05)
	plt.savefig('noise_radius_error.pdf',format='pdf')

### Plots (outliers)
if outlier_levels_number>1:
	### Orientation
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(outlier_levels,hough_orientation_results_mean[a,:,occlusion_index,noise_index],color=colors[a],label=labels[a],linestyle=linestyles[a],linewidth=linethickness[a])
		error_sup=hough_orientation_results_mean[a,:,occlusion_index,noise_index]+hough_orientation_results_std[a,:,occlusion_index,noise_index];
		error_inf=hough_orientation_results_mean[a,:,occlusion_index,noise_index]-hough_orientation_results_std[a,:,occlusion_index,noise_index];
		plt.fill_between(outlier_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('outliers [% of original surface points]',fontsize=fontsize_)
	plt.ylabel('absolute orientation error [$^\circ$]',fontsize=fontsize_)
	plt.xticks(outliers_x_ticks,color='k',size=fontsize_)
	plt.yticks(orientation_y_ticks,color='k',size=fontsize_)
	plt.legend(fontsize=fontsize_,loc='upper right')
	plt.xlim(left=min(outlier_levels))
	plt.xlim(right=max(outlier_levels))
	plt.ylim(bottom=0)
	plt.ylim(top=100)
	plt.savefig('outliers_orientation_error.pdf',format='pdf')

	### Position
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(outlier_levels,hough_position_results_mean[a,:,occlusion_index,noise_index],color=colors[a],label=labels[a],linestyle=linestyles[a],linewidth=linethickness[a])
		error_sup=hough_position_results_mean[a,:,occlusion_index,noise_index]+hough_position_results_std[a,:,occlusion_index,noise_index];
		error_inf=hough_position_results_mean[a,:,occlusion_index,noise_index]-hough_position_results_std[a,:,occlusion_index,noise_index];
		plt.fill_between(outlier_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('outliers [% of original surface points]',fontsize=fontsize_)
	plt.ylabel('absolute position error [m]',fontsize=fontsize_)
	plt.xticks(outliers_x_ticks,color='k',size=fontsize_)
	plt.yticks(color='k',size=fontsize_)
	plt.xlim(left=min(outlier_levels))
	plt.xlim(right=max(outlier_levels))
	plt.ylim(bottom=0)
	plt.savefig('outliers_position_error.pdf',format='pdf')

	### Radius
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(outlier_levels,hough_radius_results_mean[a,:,occlusion_index,noise_index],color=colors[a],label=labels[a],linestyle=linestyles[a],linewidth=linethickness[a])
		error_sup=hough_radius_results_mean[a,:,occlusion_index,noise_index]+hough_radius_results_std[a,:,occlusion_index,noise_index];
		error_inf=hough_radius_results_mean[a,:,occlusion_index,noise_index]-hough_radius_results_std[a,:,occlusion_index,noise_index];
		plt.fill_between(outlier_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('outliers [% of original surface points]',fontsize=fontsize_)
	plt.ylabel('absolute radius error [m]',fontsize=fontsize_)
	plt.xticks(outliers_x_ticks, color='k', size=fontsize_)
	plt.yticks(color='k', size=fontsize_)
	plt.xlim(left=min(outlier_levels))
	plt.xlim(right=max(outlier_levels))
	plt.ylim(bottom=0)
	plt.savefig('outliers_radius_error.pdf',format='pdf')

### Plots (occlusion)
if occlusion_levels_number>1:
	### Orientation
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(occlusion_levels,hough_orientation_results_mean[a,outlier_index,:,noise_index],color=colors[a],label=labels[a],linestyle=linestyles[a],linewidth=linethickness[a])
		error_sup=hough_orientation_results_mean[a,outlier_index,:,noise_index]+hough_orientation_results_std[a,outlier_index,:,noise_index];
		error_inf=hough_orientation_results_mean[a,outlier_index,:,noise_index]-hough_orientation_results_std[a,outlier_index,:,noise_index];
		plt.fill_between(occlusion_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('occlusion [% of original surface points]',fontsize=fontsize_)
	plt.ylabel('absolute orientation error [$^\circ$]',fontsize=fontsize_)
	plt.xticks(error_levels_x_ticks, color='k',size=fontsize_)
	plt.yticks(orientation_y_ticks,color='k',size=fontsize_)
	plt.legend(fontsize=fontsize_)
	plt.xlim(left=min(occlusion_levels))
	plt.xlim(right=max(occlusion_levels))
	plt.ylim(bottom=0)
	plt.ylim(top=100)
	plt.savefig('occlusion_orientation_error.pdf',format='pdf',pad_inches=0.8)

	### Position
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(occlusion_levels,hough_position_results_mean[a,outlier_index,:,noise_index],color=colors[a],label=labels[a],linestyle=linestyles[a],linewidth=linethickness[a])
		error_sup=hough_position_results_mean[a,outlier_index,:,noise_index]+hough_position_results_std[a,outlier_index,:,noise_index];
		error_inf=hough_position_results_mean[a,outlier_index,:,noise_index]-hough_position_results_std[a,outlier_index,:,noise_index];
		plt.fill_between(occlusion_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('occlusion [% of original surface points]',fontsize=fontsize_)
	plt.ylabel('absolute position error [m]',fontsize=fontsize_)
	plt.xticks(occlusion_x_ticks, color='k', size=fontsize_)
	plt.yticks(color='k', size=fontsize_)
	plt.xlim(left=min(occlusion_levels))
	plt.xlim(right=max(occlusion_levels))
	plt.ylim(bottom=0)
	plt.savefig('occlusion_position_error.pdf',format='pdf')

	### Radius
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(occlusion_levels,hough_radius_results_mean[a,outlier_index,:,noise_index],color=colors[a],label=labels[a],linestyle=linestyles[a],linewidth=linethickness[a])
		error_sup=hough_radius_results_mean[a,outlier_index,:,noise_index]+hough_radius_results_std[a,outlier_index,:,noise_index];
		error_inf=hough_radius_results_mean[a,outlier_index,:,noise_index]-hough_radius_results_std[a,outlier_index,:,noise_index];
		plt.fill_between(occlusion_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('occlusion [% of original cylinder surface points]',fontsize=fontsize_)
	plt.ylabel('absolute radius error [m]',fontsize=fontsize_)
	plt.xticks(occlusion_x_ticks, color='k', size=fontsize_)
	plt.yticks(color='k', size=fontsize_)
	plt.xlim(left=min(occlusion_levels))
	plt.xlim(right=max(occlusion_levels))
	plt.ylim(bottom=0)
	plt.savefig('occlusion_radius_error.pdf',format='pdf')

plt.show()

