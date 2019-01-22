/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/
#ifndef PLANEFITTINGRANSAC_H
#define PLANEFITTINGRANSAC_H
#include "plane_fitting.h"

class PlaneFittingRansac : public PlaneFitting
{
	double z_filter_min;
	double z_filter_max;
	double plane_detection_voxel_size;
	double clustering_voxel_size;
	double angle_threshold;

	pcl::VoxelGrid<PointT> grid_, grid_objects_;
 	KdTreePtr normals_tree_, clusters_tree_;
	pcl::NormalEstimation<PointT, pcl::Normal> n3d_;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_;
	pcl::ProjectInliers<PointT> proj_;
	pcl::ConvexHull<PointT> hull_;
	pcl::ExtractPolygonalPrismData<PointT> prism_;
	pcl::EuclideanClusterExtraction<PointT> pcl_cluster_;
	PointCloudT::Ptr cloud_filtered_ptr,cloud_downsampled_ptr;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr;
	pcl::PointIndices::Ptr table_inliers_ptr;
	pcl::ModelCoefficients::Ptr table_coefficients_ptr;
	boost::shared_ptr<pcl::EdgeAwarePlaneComparator<pcl::PointXYZ,pcl::Normal> > eapc;
	pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
	public:
		PlaneFittingRansac(double distance_threshold_=0.02,double cluster_tolerance_=0.02, int min_cluster_size_=100, int max_cluster_size_=25000, bool do_refine_=false, double table_z_filter_min_=0.1, double table_z_filter_max=0.5,double z_filter_min_=-10.0, double z_filter_max_=10.0, double plane_detection_voxel_size_=0.03, double clustering_voxel_size_=0.03, int inlier_threshold_=300, double angle_threshold_=3.0);
		FittingData fit(const PointCloudT::ConstPtr & point_cloud_in_,const pcl::PointCloud<pcl::Normal>::ConstPtr & point_cloud_normal_in_, float* distance_map=NULL);
		pcl::PointIndices::Ptr getConvexHull();
};


#endif // PLANEFITTINGRANSAC_H

