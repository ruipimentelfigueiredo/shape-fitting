/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/
#ifndef PLANEFITTING_H
#define PLANEFITTING_H
#include <random>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>

#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#include "fitting_data.h"

class PlaneFitting
{
	protected:
	// params
	double distance_threshold;
	double cluster_tolerance;
	double min_cluster_size;
	double max_cluster_size;

	// refine estimation
	bool do_refine;

	//aux structures
	PointCloudT::Ptr cloud_filtered;
	PointCloudT::Ptr table_cloud;
	pcl::ModelCoefficients::Ptr coefficients;
	pcl::PassThrough<PointT> pass;
	pcl::PointIndices::Ptr plane_indices;
	pcl::PointIndices::Ptr convex_hull_indices;
	pcl::search::KdTree<PointT>::Ptr tree;

	public:
		PlaneFitting(double distance_threshold_=0.02,double cluster_tolerance_=0.02, int min_cluster_size_=0.02,int max_cluster_size_=0.5, bool do_refine_=false);

	void ExtractTableTopClusters(PointCloudT::ConstPtr input_cloud, std::vector<pcl::PointIndices> & clusters_indices);

	Eigen::Matrix4f refine(const PointCloudT::ConstPtr & point_cloud_source_, const PointCloudT::ConstPtr & point_cloud_target_);
	virtual FittingData fit(const PointCloudT::ConstPtr & point_cloud_in_) = 0;
	pcl::PointIndices::Ptr getConvexHull();
	PointCloudT::Ptr getTableCloud();
	void getClustersFromPointCloud (const PointCloudT &cloud_objects, const std::vector<pcl::PointIndices> &clusters2, std::vector<PointCloudT> &clusters);
};


#endif // PLANEFITTING_H
