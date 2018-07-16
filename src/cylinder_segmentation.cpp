/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include "cylinder_segmentation.h"
boost::shared_ptr<pcl::visualization::PCLVisualizer> CylinderSegmentation::simpleVis (PointCloudT::ConstPtr cloud,  pcl::PointCloud<pcl::Normal>::ConstPtr normals, pcl::ModelCoefficients::Ptr coefficients_cylinder)
{
	
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);

  viewer->addPointCloud<PointT> (cloud,  "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
  //viewer->addPointCloudNormals<PointT, NormalT> (cloud, normals, 10, 0.01, "normals");
  viewer->addCylinder (*coefficients_cylinder);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  return (viewer);
}


Eigen::Matrix4f CylinderSegmentation::refine(const PointCloudT::ConstPtr & point_cloud_source_, const PointCloudT::ConstPtr & point_cloud_target_)
{
	pcl::IterativeClosestPoint<PointT, PointT> icp;

	icp.setInputSource(point_cloud_source_);
	icp.setInputTarget(point_cloud_target_);
	PointCloudT::Ptr Final(new PointCloudT);
	icp.align(*Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	return icp.getFinalTransformation();
}
