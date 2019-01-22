/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#ifndef CYLINDERFITTINGHOUGH_H
#define CYLINDERFITTINGHOUGH_H
#include "cylinder_fitting.h"

class CylinderFittingHough : public CylinderFitting
{
	const double curvature_epsilon=0.000001;
	private:

	std::vector<double> cos_angle;
	std::vector<double> sin_angle;
	public:

	static const unsigned int NORMAL=0;
	static const unsigned int HYBRID=1;
	// private attributes

	// Direction HT
	OrientationAccumulatorSpace gaussian_sphere;
	std::vector<double> cyl_direction_accum;

	// Circle GHT
	unsigned int angle_bins;

	double angle_step;
	unsigned int radius_bins;
	unsigned int position_bins;
	double r_step;
	double accumulator_peak_threshold;

	std::vector<std::vector<std::vector<unsigned int> > > cyl_circ_accum;

	unsigned int mode;
	bool soft_voting;
	double orientation_peak_threshold;

	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;

	// private methods
	std::tuple<Eigen::Vector3d, double> findCylinderDirection(const pcl::PointCloud<pcl::Normal>::ConstPtr & normal_cloud, const PointCloudT::ConstPtr & point_cloud_in_);
	Eigen::Matrix<double,5,1> findCylinderPositionRadius(const PointCloudT::ConstPtr & point_cloud_in_, const pcl::PointCloud<pcl::Normal>::ConstPtr & normal_cloud);

	public:
		CylinderFittingHough(const OrientationAccumulatorSpace & gaussian_sphere_, unsigned int angle_bins_=30,unsigned int radius_bins_=10,unsigned int position_bins_=10,double min_radius_=0.01,double max_radius_=0.1, double accumulator_peak_threshold_=0.2, unsigned int mode_=0, bool do_refine_=false, bool soft_voting_=true, double orientation_peak_threshold_=0.003);

		FittingData fit(const PointCloudT::ConstPtr & point_cloud_in_,const pcl::PointCloud<pcl::Normal>::ConstPtr & cloud_normal_in_);


};

#endif // CYLINDERFITTINGHOUGH_H
