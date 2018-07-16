/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/
#ifndef CYLINDERFITTINGRANSAC_H
#define CYLINDERFITTINGRANSAC_H
#include "cylinder_fitting.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/ransac.h>

class CylinderFittingRansac : public CylinderFitting
{
	// Parameters
	float normal_distance_weight; 
	unsigned int max_iterations; 
	float distance_threshold;

	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

	public:
		CylinderFittingRansac(float normal_distance_weight_=0.1, unsigned int max_iterations_=1000, float distance_threshold_=0.1, float min_radius_=0.01,float max_radius_=0.1, bool do_refine_=false);
		FittingData fit(const PointCloudT::ConstPtr & point_cloud_in_);
};

#endif // CYLINDERFITTINGRANSAC_H
