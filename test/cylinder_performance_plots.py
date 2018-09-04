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
home=path.expanduser('~/ws/src/shape_detection_fitting/lib/shape-fitting/dataset/cylinder_outliers/results')
#home=path.expanduser('~/')
def to_percent(y, position):
    # Ignore the passed in position. This has the effect of scaling the default
    # tick locations.
    s = str(100 * y)

    # The percent symbol needs escaping in latex
    if plt.rcParams['text.usetex'] is True:
        return s #+ r'$\%$'
    else:
        return s #+ '%'

total_algorithms=10
heights=1
radii=1
iterations=1000
ground_truth_size=heights*radii

outlier_levels_number=9
noise_levels_number=1
occlusion_levels_number=1

noise_index=0
outlier_index=0
occlusion_index=0

alpha_=0.1
fontsize_=20

error_levels=[0,10,20,30,40,50,60,70,80,90,100]
outlier_levels=[0,25,50,75,100,125,150,175,200]
occlusion_levels=[0,10,20,30,40,50,60,70,80]


colors=['green','green','blue','blue','red','red','black','black','orange','orange','brown','brown','aqua','aqua','yellow','yellow','purple','purple']
labels=['Rabbani et al.','Rabbani et al. (soft-voting)','Deterministic Orientations','Deterministic Orientations (soft-voting)','Ours (Unbiased)','Ours (Unbiased and soft-voting)','Ours (Weak Vertical-Bias)','Ours (Weak Vertical-Bias and soft-voting)','Ours (Strong Vertical-Bias)','Ours (Strong Vertical-Bias and soft-voting)']
linestyles = ['-', '--','-', '--','-', '--','-', '--','-', '--']
linethickness=[1, 2, 3, 4, 5]

algorithms=[0,5,7,9]

hough_orientation_results=[]
hough_position_results=[]
hough_radius_results=[]
for a in range(0,total_algorithms):
	#ORIENTATION
	hough_orientation_file=open(home + "/orientation_noise_" + str(a) + ".txt", "r")
	for line in hough_orientation_file:
  		hough_orientation_results.append(180.0*float(line)/math.pi)

	#POSITION
	hough_position_file=open(home + "/position_noise_" + str(a) + ".txt", "r")
	for line in hough_position_file:
  		hough_position_results.append(float(line))

	#POSITION
	hough_radius_file=open(home + "/radius_noise_" + str(a) + ".txt", "r")
	for line in hough_radius_file:
  		hough_radius_results.append(float(line))

hough_orientation_results = np.array(hough_orientation_results).reshape(total_algorithms,iterations,outlier_levels_number,occlusion_levels_number,ground_truth_size,noise_levels_number)
hough_position_results = np.array(hough_position_results).reshape(total_algorithms,iterations,outlier_levels_number,occlusion_levels_number,ground_truth_size,noise_levels_number)
hough_radius_results = np.array(hough_radius_results).reshape(total_algorithms,iterations,outlier_levels_number,occlusion_levels_number,ground_truth_size,noise_levels_number)

# compute orientation average and standard deviation
hough_orientation_results_mean = np.mean(hough_orientation_results, axis=(1,4))
hough_orientation_results_std = np.std(hough_orientation_results, axis=(1,4))

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
		plt.plot(error_levels,hough_orientation_results_mean[a,outlier_index,occlusion_index,:],color=colors[a],label=labels[a],linestyle=linestyles[a])
		error_sup=hough_orientation_results_mean[a,outlier_index,occlusion_index,:]+hough_orientation_results_std[a,outlier_index,occlusion_index,:];
		error_inf=hough_orientation_results_mean[a,outlier_index,occlusion_index,:]-hough_orientation_results_std[a,outlier_index,occlusion_index,:];
		plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('noise standard deviation [% of cylinder radius]',fontsize=fontsize_)
	plt.ylabel('absolute orientation error [$^\circ$]',fontsize=fontsize_)
	plt.xticks(color='k', size=fontsize_)
	plt.yticks(color='k', size=fontsize_)
	plt.legend(fontsize=fontsize_)
	plt.savefig('noise_orientation_error.pdf',format='pdf',pad_inches=0.8)

	### Position
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(error_levels,hough_position_results_mean[a,outlier_index,occlusion_index,:],color=colors[a],label=labels[a],linestyle=linestyles[a])
		error_sup=hough_position_results_mean[a,outlier_index,occlusion_index,:]+hough_position_results_std[a,outlier_index,occlusion_index,:];
		error_inf=hough_position_results_mean[a,outlier_index,occlusion_index,:]-hough_position_results_std[a,outlier_index,occlusion_index,:];
		plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('noise standard deviation [% of cylinder radius]',fontsize=fontsize_)
	plt.ylabel('absolute position error [m]',fontsize=fontsize_)
	plt.xticks(color='k', size=fontsize_)
	plt.yticks(color='k', size=fontsize_)
	plt.savefig('noise_position_error.pdf',format='pdf')

	### Radius
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(error_levels,hough_radius_results_mean[a,outlier_index,occlusion_index,:],color=colors[a],label=labels[a],linestyle=linestyles[a])
		error_sup=hough_radius_results_mean[a,outlier_index,occlusion_index,:]+hough_radius_results_std[a,outlier_index,occlusion_index,:];
		error_inf=hough_radius_results_mean[a,outlier_index,occlusion_index,:]-hough_radius_results_std[a,outlier_index,occlusion_index,:];
		plt.fill_between(error_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('noise standard deviation [% of cylinder radius]',fontsize=fontsize_)
	plt.ylabel('absolute radius error [m]',fontsize=fontsize_)
	plt.xticks(color='k', size=fontsize_)
	plt.yticks(color='k', size=fontsize_)
	plt.savefig('noise_radius_error.pdf',format='pdf')


### Plots (outliers)
if outlier_levels_number>1:
	### Orientation
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(outlier_levels,hough_orientation_results_mean[a,:,occlusion_index,noise_index],color=colors[a],label=labels[a],linestyle=linestyles[a])
		error_sup=hough_orientation_results_mean[a,:,occlusion_index,noise_index]+hough_orientation_results_std[a,:,occlusion_index,noise_index];
		error_inf=hough_orientation_results_mean[a,:,occlusion_index,noise_index]-hough_orientation_results_std[a,:,occlusion_index,noise_index];
		plt.fill_between(outlier_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('noise standard deviation [% of cylinder radius]',fontsize=fontsize_)
	plt.ylabel('absolute orientation error [$^\circ$]',fontsize=fontsize_)
	plt.xticks(color='k', size=fontsize_)
	plt.yticks(color='k', size=fontsize_)
	plt.legend(fontsize=fontsize_)
	plt.savefig('noise_orientation_error.pdf',format='pdf',pad_inches=0.8)

	### Position
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(outlier_levels,hough_position_results_mean[a,:,occlusion_index,noise_index],color=colors[a],label=labels[a],linestyle=linestyles[a])
		error_sup=hough_position_results_mean[a,:,occlusion_index,noise_index]+hough_position_results_std[a,:,occlusion_index,noise_index];
		error_inf=hough_position_results_mean[a,:,occlusion_index,noise_index]-hough_position_results_std[a,:,occlusion_index,noise_index];
		plt.fill_between(outlier_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('noise standard deviation [% of cylinder radius]',fontsize=fontsize_)
	plt.ylabel('absolute position error [m]',fontsize=fontsize_)
	plt.xticks(color='k', size=fontsize_)
	plt.yticks(color='k', size=fontsize_)
	plt.savefig('noise_position_error.pdf',format='pdf')

	### Radius
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(outlier_levels,hough_radius_results_mean[a,:,occlusion_index,noise_index],color=colors[a],label=labels[a],linestyle=linestyles[a])
		error_sup=hough_radius_results_mean[a,:,occlusion_index,noise_index]+hough_radius_results_std[a,:,occlusion_index,noise_index];
		error_inf=hough_radius_results_mean[a,:,occlusion_index,noise_index]-hough_radius_results_std[a,:,occlusion_index,noise_index];
		plt.fill_between(outlier_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('noise standard deviation [% of cylinder radius]',fontsize=fontsize_)
	plt.ylabel('absolute radius error [m]',fontsize=fontsize_)
	plt.xticks(color='k', size=fontsize_)
	plt.yticks(color='k', size=fontsize_)
	plt.savefig('noise_radius_error.pdf',format='pdf')



### Plots (occlusion)
if occlusion_levels_number>1:
	### Orientation
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(occlusion_levels,hough_orientation_results_mean[a,outlier_index,:,noise_index],color=colors[a],label=labels[a],linestyle=linestyles[a])
		error_sup=hough_orientation_results_mean[a,outlier_index,:,noise_index]+hough_orientation_results_std[a,outlier_index,:,noise_index];
		error_inf=hough_orientation_results_mean[a,outlier_index,:,noise_index]-hough_orientation_results_std[a,outlier_index,:,noise_index];
		plt.fill_between(occlusion_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('occlusion levels [% of original cylinder surface points]',fontsize=fontsize_)
	plt.ylabel('absolute orientation error [$^\circ$]',fontsize=fontsize_)
	plt.xticks(color='k', size=fontsize_)
	plt.yticks(color='k', size=fontsize_)
	plt.legend(fontsize=fontsize_)
	plt.savefig('occlusion_orientation_error.pdf',format='pdf')

	### Position
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(occlusion_levels,hough_position_results_mean[a,outlier_index,:,noise_index],color=colors[a],label=labels[a],linestyle=linestyles[a])
		error_sup=hough_position_results_mean[a,outlier_index,:,noise_index]+hough_position_results_std[a,outlier_index,:,noise_index];
		error_inf=hough_position_results_mean[a,outlier_index,:,noise_index]-hough_position_results_std[a,outlier_index,:,noise_index];
		plt.fill_between(occlusion_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('occlusion [% of original surface points]',fontsize=fontsize_)
	plt.ylabel('absolute position error [m]',fontsize=fontsize_)
	plt.xticks(color='k', size=fontsize_)
	plt.yticks(color='k', size=fontsize_)
	plt.savefig('occlusion_position_error.pdf',format='pdf')

	### Radius
	plt.figure(figsize=(8, 6))
	for a in algorithms:
		plt.plot(occlusion_levels,hough_radius_results_mean[a,outlier_index,:,noise_index],color=colors[a],label=labels[a],linestyle=linestyles[a])
		error_sup=hough_radius_results_mean[a,outlier_index,:,noise_index]+hough_radius_results_std[a,outlier_index,:,noise_index];
		error_inf=hough_radius_results_mean[a,outlier_index,:,noise_index]-hough_radius_results_std[a,outlier_index,:,noise_index];
		plt.fill_between(occlusion_levels,error_sup,error_inf,where=error_inf<=error_sup,interpolate=True,alpha=alpha_,color=colors[a],linestyle=linestyles[a])

	manager = plt.get_current_fig_manager()
	manager.resize(*manager.window.maxsize())
	plt.xlabel('occlusion [% of original cylinder surface points]',fontsize=fontsize_)
	plt.ylabel('absolute radius error [m]',fontsize=fontsize_)

	plt.xticks(color='k', size=fontsize_)
	plt.yticks(color='k', size=fontsize_)
	plt.savefig('occlusion_radius_error.pdf',format='pdf')

plt.show()

