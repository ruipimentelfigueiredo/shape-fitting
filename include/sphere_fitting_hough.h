/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#ifndef SPHEREFITTINGHOUGH_H
#define SPHEREFITTINGHOUGH_H

#include "sphere_fitting.h"

class SphereFittingHough : SphereFitting
{
        public:

	// Direction HT
	boost::shared_ptr<OrientationAccumulatorSpace> gaussian_sphere;

	unsigned int angle_bins;

	double angle_step;
	unsigned int radius_bins;
	unsigned int position_bins;
	double r_step;
	double accumulator_peak_threshold;
	std::vector<std::vector<std::vector<std::vector<double> > > > sphere_accum;
	bool soft_voting;

	// Normal estimation
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;// (new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::search::KdTree<PointT>::Ptr tree;

	std::vector<double> radii;
	public:
		SphereFittingHough(const OrientationAccumulatorSpace & gaussian_sphere_, unsigned int position_bins_=30,unsigned int radius_bins_=10,double min_radius_=0.01,double max_radius_=0.1, double accumulator_peak_threshold_=0.2, bool do_refine_=false, bool soft_voting_=false);
		~SphereFittingHough() {};
		FittingData fit(const PointCloudT::ConstPtr & point_cloud_in_);
};

#endif // SPHEREFITTINGHOUGH_H
