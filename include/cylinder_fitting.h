/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/
#ifndef CylinderFitting_H
#define CylinderFitting_H
#include <random>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/registration/icp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/integral_image_normal.h>
#include <Eigen/Geometry>

#include "gaussian_sphere.h"
#include "spherical_grid.h"
#include "fitting_data.h"

class CylinderFitting
{
	protected:
	// params
	float min_radius;
	float max_radius;

	// refine estimation
	bool do_refine;

	// Normal estimation
	//pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::search::KdTree<PointT>::Ptr tree; 

	PointCloudT::Ptr cloud_filtered;
	PointCloudT::Ptr transformed_cloud;
	pcl::PointCloud<pcl::Normal>::Ptr transformed_normals;
	pcl::PointIndices::Ptr inliers_cylinder;

	pcl::PassThrough<PointT> pass;

	public:
		CylinderFitting(float min_radius_=0.01,float max_radius_=0.1, bool do_refine_=false) : 
			min_radius(min_radius_), 
			max_radius(max_radius_), 
			do_refine(do_refine_),
			tree(new pcl::search::KdTree<PointT> ()),	
			cloud_filtered(new pcl::PointCloud<PointT>),
			transformed_cloud(new pcl::PointCloud<PointT> ()),
			transformed_normals(new pcl::PointCloud<pcl::Normal>()),
			inliers_cylinder(new pcl::PointIndices)
		{};

	Eigen::Matrix4f refine(const PointCloudT::ConstPtr & point_cloud_source_, const PointCloudT::ConstPtr & point_cloud_target_);
	virtual FittingData fit(const PointCloudT::ConstPtr & point_cloud_in_, const pcl::PointCloud<pcl::Normal>::ConstPtr & cloud_) = 0;
};

#endif // CylinderFitting_H

