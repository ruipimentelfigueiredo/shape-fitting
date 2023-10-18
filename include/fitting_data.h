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
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

typedef pcl::PointCloud<NormalT> NormalCloudT;
typedef pcl::search::KdTree<pcl::PointXYZ> KdTree;
typedef pcl::search::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;

class PCL_EXPORTS FittingException : public pcl::PCLException
{
  public:
	FittingException(const std::string &error_description, const char *file_name = NULL, const char *function_name = NULL, unsigned line_number = 0) : pcl::PCLException(error_description, file_name, function_name, line_number) {}
};

class VisualizeFittingData
{
	std::vector<std::string> coordinate_systems;

  public:
	VisualizeFittingData() : viewer(new pcl::visualization::PCLVisualizer("3D Viewer"))
	{
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addCoordinateSystem(0.3);
		//viewer->initCameraParameters ();
		//viewer->setCameraPosition(0,0,0,0,-1,0,-1,0,0);
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	void addPointCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_other, std::string &id)
	{
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_other);
		viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_other, rgb, id);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, id);
	}

	void addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_other, std::string &id, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> &rgb)
	{
		viewer->addPointCloud<pcl::PointXYZ>(point_cloud_other, rgb, id);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, id);
	}

	void spin(double time)
	{
		viewer->spinOnce(time);
	}

	void clear()
	{
		viewer->removeAllShapes();
		viewer->removeAllPointClouds();
	}

	void addCoordinateSystem(double scale, Eigen::Affine3d transf, std::string name)
	{
		viewer->addCoordinateSystem(scale, transf.cast<float>(), name);
		coordinate_systems.push_back(name);
	}

	void removeCoordinateSystem(std::string name)
	{
		viewer->removeCoordinateSystem(name);
	}

	void removeCoordinateSystems()
	{
		for (unsigned int i = 0; i < coordinate_systems.size(); ++i)
		{
			this->removeCoordinateSystem(coordinate_systems[i]);
			//this->spin(0);
		}

		coordinate_systems.clear();
	}
};

class FittingData
{
  public:
	static double fitting_threshold;
	FittingData() : confidence(1.0), type(FittingData::CYLINDER) 
	{}

	FittingData(
		const Eigen::VectorXf &parameters_, 
		const double &confidence_, 
		const unsigned int type_, 
		PointCloudT::Ptr inliers_= PointCloudT::Ptr(), 
		PointCloudT::Ptr outliers_ = PointCloudT::Ptr(),
		PointCloudT::Ptr contour_ = PointCloudT::Ptr()) : 
			parameters(parameters_), 
			confidence(confidence_), 
			type(type_), 
			inliers(inliers_), 
			outliers(outliers_),
			contour(contour_),
			id(++id_count)
	{}

	static const unsigned int CYLINDER = 0;
	static const unsigned int SPHERE = 1;
	static const unsigned int PLANE = 2;
	static const unsigned int OTHER = 3;
	Eigen::VectorXf parameters;
	double confidence;
	unsigned int type;
	PointCloudT::Ptr inliers;
	PointCloudT::Ptr outliers;
	PointCloudT::Ptr contour;

	int id;

	static int id_count;
	friend ostream &operator<<(std::ostream &os, const FittingData &ft)
	{
		for (unsigned int i = 0; i < ft.parameters.size(); ++i)
		{
			os << ft.parameters[i] << " ";
		}
		return os;
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize(std::vector<FittingData> fitting_data)
	{
		// ---------------------------------------------
		// -----Open 3D viewer and add point clouds-----
		// ---------------------------------------------
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addCoordinateSystem(1.0);
		viewer->initCameraParameters();
		//pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);

		for (unsigned int i = 0; i < fitting_data.size(); ++i)
		{
			viewer->addPointCloud<PointT>(fitting_data[i].inliers, std::to_string(fitting_data[i].id).c_str());
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, std::to_string(fitting_data[i].id).c_str());
			pcl::ModelCoefficients coeffs;
			for (unsigned int p = 0; p < fitting_data[i].parameters.size(); ++p)
			{
				coeffs.values.push_back(fitting_data[i].parameters[p]);
			}

			if (fitting_data[i].type == FittingData::CYLINDER)
			{
				viewer->addCylinder(coeffs);
			}
			else if (fitting_data[i].type == FittingData::SPHERE)
			{
				viewer->addSphere(coeffs);
			}
			else if (fitting_data[i].type == FittingData::PLANE)
			{
				viewer->addPlane(coeffs);
			}
		}

		return (viewer);
	}

	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize()
	{
		visualize_fitting_data.viewer->removeAllShapes();
		visualize_fitting_data.viewer->removeAllPointClouds();
		visualize_fitting_data.viewer->addPointCloud<PointT>(this->inliers, std::to_string(this->id).c_str());
		visualize_fitting_data.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, std::to_string(this->id).c_str());
		pcl::ModelCoefficients coeffs;
		for (unsigned int p = 0; p < this->parameters.size(); ++p)
		{
			coeffs.values.push_back(this->parameters[p]);
		}
		if (this->type == FittingData::CYLINDER)
		{
			visualize_fitting_data.viewer->addCylinder(coeffs);
		}
		else if (this->type == FittingData::SPHERE)
		{
			visualize_fitting_data.viewer->addSphere(coeffs);
		}
		else if (this->type == FittingData::PLANE)
		{
			visualize_fitting_data.viewer->addPlane(coeffs);
		}
		visualize_fitting_data.viewer->spinOnce(1);
		return (visualize_fitting_data.viewer);
	}*/

	void visualize(boost::shared_ptr<VisualizeFittingData> visualizer)
	{
		//visualizer->viewer->addPointCloud<PointT> (this->inliers,  std::to_string(this->id).c_str());
		//visualizer->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,  std::to_string(this->id).c_str());
		pcl::ModelCoefficients coeffs;
		for (unsigned int p = 0; p < this->parameters.size(); ++p)
		{
			if (this->type == FittingData::CYLINDER && (p > 6))
				break;
			coeffs.values.push_back(this->parameters[p]);
		}
		if (this->type == FittingData::CYLINDER)
		{
			//visualizer->viewer->addCylinder (coeffs,std::to_string(this->id).c_str(),0,255,0);
			pcl::PointXYZ arrow_begin(this->parameters[0],
									  this->parameters[1],
									  this->parameters[2]);

			pcl::PointXYZ arrow_end(this->parameters[0] + this->parameters[7] * this->parameters[3],
									this->parameters[1] + this->parameters[7] * this->parameters[4],
									this->parameters[2] + this->parameters[7] * this->parameters[5]);

			visualizer->viewer->addArrow(arrow_begin, arrow_end, 255, 0, 0, false, std::to_string(this->id).c_str());
		}
		else if (this->type == FittingData::SPHERE)
		{
			visualizer->viewer->addSphere(coeffs, std::to_string(this->id).c_str());
		}
		else if (this->type == FittingData::PLANE)
		{
			//visualizer->viewer->addPlane (coeffs);
			std::vector<pcl::Vertices> hull_polygons;
			PointCloudT::Ptr hull_points(new PointCloudT);
			PointCloudT::Ptr output_cloud(new PointCloudT);
			int dim = 2;
			hull_points = calculateHull(hull_polygons, dim, this->inliers, 1.0);
			cropToHull(output_cloud, this->inliers, hull_points, hull_polygons, dim);
			visualizer->viewer->addPolygonMesh<PointT>(hull_points, hull_polygons, std::to_string(this->id).c_str());

			pcl::PointXYZ normal_dir(this->parameters[0], this->parameters[1], this->parameters[2]);
			double distance = this->parameters[3];
			pcl::PointXYZ plane_point(0, 0, distance);
			pcl::PointXYZ arrow_end(this->parameters[0], this->parameters[1], this->parameters[2] + distance);
			visualizer->viewer->addArrow(plane_point, arrow_end, 255, 255, 0, false, std::to_string(this->id + 100).c_str());
		}
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize(PointCloudT::Ptr point_cloud_other,boost::shared_ptr<VisualizeFittingData> visualizer)
	{
		visualizer->viewer->removeAllShapes();
		visualizer->viewer->removeAllPointClouds();

		pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb1(this->inliers, 255, 0, 0);
		visualizer->viewer->addPointCloud<PointT>(this->inliers, rgb1, std::to_string(this->id).c_str());
		pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb2(point_cloud_other, 0, 0, 255);

		visualizer->viewer->addPointCloud<PointT>(point_cloud_other, rgb2, "original cloud");
		visualizer->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, std::to_string(this->id).c_str());
		visualizer->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original cloud");
		pcl::ModelCoefficients coeffs;
		for (unsigned int p = 0; p < this->parameters.size(); ++p)
		{
			if (this->type == FittingData::CYLINDER && (p > 6))
				break;

			coeffs.values.push_back(this->parameters[p]);
		}
		if (this->type == FittingData::CYLINDER)
		{
			visualizer->viewer->addCylinder(coeffs);
		}
		else if (this->type == FittingData::SPHERE)
		{
			visualizer->viewer->addSphere(coeffs);
		}
		else if (this->type == FittingData::PLANE)
		{
			//visualizer->viewer->addPlane (coeffs);
			std::vector<pcl::Vertices> hull_polygons;
			PointCloudT::Ptr hull_points(new PointCloudT);
			PointCloudT::Ptr output_cloud(new PointCloudT);
			int dim = 2;
			hull_points = calculateHull(hull_polygons, dim, this->inliers, 1.0);
			cropToHull(output_cloud, this->inliers, hull_points, hull_polygons, dim);
			visualizer->viewer->addPolygonMesh<PointT>(hull_points, hull_polygons, "hull surface");
		}
		visualizer->viewer->spinOnce(1);
		return (visualizer->viewer);
	}

	PointCloudT::Ptr calculateHull(std::vector<pcl::Vertices> &polygons, int &dim, PointCloudT::Ptr cloud, double alpha)
	{
		pcl::ConcaveHull<PointT> hull_calculator;
		PointCloudT::Ptr hull(new PointCloudT);
		hull_calculator.setInputCloud(cloud);
		hull_calculator.setAlpha(alpha);
		hull_calculator.reconstruct(*hull, polygons);

		dim = hull_calculator.getDimension();
		return hull;
	}

	void cropToHull(PointCloudT::Ptr output, PointCloudT::Ptr input, PointCloudT::Ptr hull_cloud, std::vector<pcl::Vertices> const &polygons, int dim)
	{
		pcl::CropHull<PointT> crop_filter;
		crop_filter.setInputCloud(input);
		crop_filter.setHullCloud(hull_cloud);
		crop_filter.setHullIndices(polygons);
		crop_filter.setDim(dim);
		crop_filter.filter(*output);
	}

	Eigen::Affine3d computeReferenceFrame()
	{
		Eigen::Vector3d view_direction(Eigen::Vector3d::UnitZ());										 // view_direction
		Eigen::Vector3d plane_normal_dir(this->parameters[0], this->parameters[1], this->parameters[2]); // normal
		plane_normal_dir.normalize();

		double distance = this->parameters[3];
		if (plane_normal_dir.dot(view_direction) > 0)
			plane_normal_dir = -plane_normal_dir;
		// Align view_direction with normal
		double angle = acos(view_direction.dot(plane_normal_dir.normalized()));
		Eigen::Vector3d cross = view_direction.cross(plane_normal_dir.normalized()).normalized();

		Eigen::Matrix3d rotation;
		rotation = Eigen::AngleAxisd(angle, cross);

		Eigen::Vector4d position(0, 0, fabs(distance), 1);

		//Eigen::Vector4d position(-distance/this->parameters[0],-distance/this->parameters[1],-distance/this->parameters[2],1);

		Eigen::Affine3d transform(Eigen::Matrix4d::Identity());
		transform.matrix().block(0, 0, 3, 3) = rotation;
		//transform.matrix().col(3)=position;
		return transform;
	}
};

#endif // FITTINGDATA_H
