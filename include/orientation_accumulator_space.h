/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/
#ifndef ORIENTATIONACCUMULATORSPACE_H
#define ORIENTATIONACCUMULATORSPACE_H
#include "boost/shared_ptr.hpp"
#include <boost/thread/thread.hpp>
#include "fitting_data.h"
class OrientationAccumulatorSpace
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> sphereViz (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
	{
	  // --------------------------------------------------------
	  // -----Open 3D viewer and add point cloud and normals-----
	  // --------------------------------------------------------
	  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	  viewer->setBackgroundColor (0, 0, 0);
	  viewer->addPointCloud<pcl::PointXYZ> (cloud,  "sample cloud");
	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	  viewer->addCoordinateSystem (1.0);
	  viewer->initCameraParameters ();
	  return (viewer);
	}
	public:
		OrientationAccumulatorSpace() {};
		OrientationAccumulatorSpace(const unsigned int & gaussian_sphere_points_num_=900, const unsigned int & orientation_accumulators_num_=10):gaussian_sphere_points_num(gaussian_sphere_points_num_), orientation_accumulators_num(orientation_accumulators_num_)
		{};


		void visualizeGaussianSphere(unsigned int sphere_index=0)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_pcl(new pcl::PointCloud<pcl::PointXYZ> ());
			for(unsigned int i=0; i<accumulator_space[sphere_index].size();++i)
			{
				sphere_pcl->points.push_back(pcl::PointXYZ(accumulator_space[sphere_index][i][0],accumulator_space[sphere_index][i][1],accumulator_space[sphere_index][i][2]));
			}
	  		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	    		viewer = sphereViz(sphere_pcl);
	  		while (!viewer->wasStopped ())
	  		{
	    			viewer->spinOnce (100);
	    			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	  		}
		}

		//virtual ~OrientationAccumulatorSpace() {};
		virtual const std::vector<Eigen::Vector3d> & getOrientationAccumulatorSpace() { return accumulator_space[0];};
		unsigned int gaussian_sphere_points_num;
		unsigned int orientation_accumulators_num;
		std::vector<std::vector<Eigen::Vector3d> > accumulator_space;
};
#endif // ORIENTATIONACCUMULATORSPACE_H
