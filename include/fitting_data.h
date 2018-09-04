/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/
#ifndef FITTINGDATA_H
#define FITTINGDATA_H
#include <iostream>  
#include <pcl/exceptions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
// DEFINE THE TYPES USED
typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<NormalT> NormalCloudT;
typedef pcl::search::KdTree<pcl::PointXYZ> KdTree;
typedef pcl::search::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;

class PCL_EXPORTS FittingException : public pcl::PCLException
{
  public:
    FittingException (const std::string& error_description, const char* file_name = NULL, const char* function_name = NULL, unsigned line_number = 0) : 
	pcl::PCLException (error_description, file_name, function_name, line_number) {}
      
};

class VisualizeFittingData
{

	public:
		VisualizeFittingData() : viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"))
		{
			viewer->setBackgroundColor (255, 255, 255);
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
			viewer->addCoordinateSystem (1.0);
			viewer->initCameraParameters ();
		}
		
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};

class FittingData
{
	public:
	FittingData() {};
	FittingData(const Eigen::VectorXf & parameters_, const double & confidence_, const unsigned int type_, PointCloudT::Ptr point_cloud_): 
		parameters(parameters_),confidence(confidence_), type(type_), point_cloud(point_cloud_),id(id_count++)
	{
	
	}

	static const unsigned int CYLINDER=0;
	static const unsigned int SPHERE=1;
	static const unsigned int PLANE=2;

 	Eigen::VectorXf parameters;
	double confidence;
	unsigned int type;
	PointCloudT::Ptr point_cloud;
	int id;
	static VisualizeFittingData visualize_fitting_data;
	static int id_count;
    	friend ostream& operator<<(std::ostream& os, const FittingData& ft)  
	{  
		for(unsigned int i=0; i<ft.parameters.size();++i)
		{
			os << ft.parameters[i] << " ";  
		}
		return os;  
	}  

	boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize (std::vector<FittingData> fitting_data, std::vector<pcl::ModelCoefficients::Ptr> coefficients)
	{
		// ---------------------------------------------
		// -----Open 3D viewer and add point clouds-----
		// ---------------------------------------------
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
		viewer->setBackgroundColor (255, 255, 255);
		//pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);

		for(unsigned int i=0; i<fitting_data.size();++i)
		{
			viewer->addPointCloud<PointT> (fitting_data[i].point_cloud,  std::to_string(fitting_data[i].id).c_str());
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,  std::to_string(fitting_data[i].id).c_str());
			pcl::ModelCoefficients coeffs;
			for(unsigned int p=0; p<fitting_data[i].parameters.size();++p)
			{
				coeffs.values.push_back(fitting_data[i].parameters[p]);
			}

			if(fitting_data[i].type==FittingData::CYLINDER)
			{
				viewer->addCylinder (coeffs);
			}
			else if(fitting_data[i].type==FittingData::SPHERE)
			{
				viewer->addSphere (coeffs);
			}
			else if(fitting_data[i].type==FittingData::PLANE)
			{
				viewer->addPlane (coeffs);
			}
		}
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
		return (viewer);
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize ()
	{
		visualize_fitting_data.viewer->removeAllShapes();
		visualize_fitting_data.viewer->removeAllPointClouds();
		visualize_fitting_data.viewer->addPointCloud<PointT> (this->point_cloud,  std::to_string(this->id).c_str());
		visualize_fitting_data.viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,  std::to_string(this->id).c_str());
		pcl::ModelCoefficients coeffs;
		for(unsigned int p=0; p<this->parameters.size();++p)
		{
			coeffs.values.push_back(this->parameters[p]);
		}
		if(this->type==FittingData::CYLINDER)
		{
			visualize_fitting_data.viewer->addCylinder (coeffs);
		}
		else if(this->type==FittingData::SPHERE)
		{
			visualize_fitting_data.viewer->addSphere (coeffs);
		}
		else if(this->type==FittingData::PLANE)
		{
			visualize_fitting_data.viewer->addPlane (coeffs);
		}
		visualize_fitting_data.viewer->spinOnce(100);
		return (visualize_fitting_data.viewer);
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize (PointCloudT::Ptr point_cloud_other)
	{
		visualize_fitting_data.viewer->removeAllShapes();
		visualize_fitting_data.viewer->removeAllPointClouds();
		pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb1(this->point_cloud, 255, 0, 0);
		visualize_fitting_data.viewer->addPointCloud<PointT> (this->point_cloud, rgb1, std::to_string(this->id).c_str());
		pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb2(point_cloud_other, 0, 255, 0);
		visualize_fitting_data.viewer->addPointCloud<PointT> (point_cloud_other,rgb2, "original cloud");
		visualize_fitting_data.viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,  std::to_string(this->id).c_str());
		pcl::ModelCoefficients coeffs;
		for(unsigned int p=0; p<this->parameters.size();++p)
		{
			if(this->type==FittingData::CYLINDER&& (p>6)) break;
			
			coeffs.values.push_back(this->parameters[p]);
		}
		if(this->type==FittingData::CYLINDER)
		{
			visualize_fitting_data.viewer->addCylinder (coeffs);
		}
		else if(this->type==FittingData::SPHERE)
		{
			visualize_fitting_data.viewer->addSphere (coeffs);
		}
		else if(this->type==FittingData::PLANE)
		{
			//visualize_fitting_data.viewer->addPlane (coeffs);
 			std::vector<pcl::Vertices> hull_polygons;
 			PointCloudT::Ptr hull_points (new PointCloudT);
 			PointCloudT::Ptr output_cloud (new PointCloudT);
			int dim=2;
			hull_points = calculateHull (hull_polygons, dim,this->point_cloud, 1.0);
 			cropToHull (output_cloud, this->point_cloud, hull_points, hull_polygons, dim);
			visualize_fitting_data.viewer->addPolygonMesh<PointT> (hull_points, hull_polygons, "hull surface");
		}
		visualize_fitting_data.viewer->spinOnce(100);
		return (visualize_fitting_data.viewer);
	}

	PointCloudT::Ptr calculateHull (std::vector<pcl::Vertices>& polygons, int& dim, PointCloudT::Ptr cloud, double alpha)
	{
		pcl::ConcaveHull<PointT> hull_calculator;
		PointCloudT::Ptr hull (new PointCloudT);
		hull_calculator.setInputCloud (cloud);
		hull_calculator.setAlpha (alpha);
		hull_calculator.reconstruct (*hull, polygons);

		dim = hull_calculator.getDim();
		return hull;
	}


	void cropToHull (PointCloudT::Ptr output, PointCloudT::Ptr input, PointCloudT::Ptr hull_cloud, std::vector<pcl::Vertices> const& polygons, int dim)
	{
		pcl::CropHull<PointT> crop_filter;
		crop_filter.setInputCloud (input);
		crop_filter.setHullCloud (hull_cloud);
		crop_filter.setHullIndices (polygons);
		crop_filter.setDim (dim);

		crop_filter.filter (*output);
	}

};


#endif // FITTINGDATA_H
