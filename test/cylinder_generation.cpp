/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/project_inliers.h>
#include "helpers.h"

int main (int argc, char** argv)
{
	/* Program arguments check */
	if (argc < 6)
	{
		std::cout << "invalid number of arguments: dataset_dir iterations heights radii noise outliers height_sampling_rate angle_sampling_rate"<< std::endl;
		exit(-1);
	}

	std::random_device rd{};
	std::mt19937 gen{rd()};
	std::normal_distribution<> d1{0,  0.005};
	std::normal_distribution<> d2{0,  0.005};
	std::normal_distribution<> d3{1.0,0.005};

	std::string dataset_dir = std::string(argv[1]);
	std::cout << "dataset_dir: " << dataset_dir << std::endl;

	std::string ground_truth_dir = dataset_dir+"/ground_truth/";
	std::string point_clouds_dir = dataset_dir+"/point_clouds/";
	createDirectory(ground_truth_dir);
	createDirectory(point_clouds_dir);

	unsigned int iterations=atoi(argv[2]);
	std::cout << "iterations: " << iterations << std::endl;

	static std::string heights_ = std::string(argv[3]);
	std::cout << "heights: " << heights_ << std::endl;

	static std::string radii_ = std::string(argv[4]); 
	std::cout << "radii: " << radii_ << std::endl;

	static std::string noise_levels_ = std::string(argv[5]);
	std::cout << "noise: " << noise_levels_ << std::endl;

	static std::string outlier_levels_ = std::string(argv[6]);
	std::cout << "outliers: " << outlier_levels_ << std::endl;

	static std::string occlusion_levels_ = std::string(argv[7]);
	std::cout << "occlusion_levels: " << occlusion_levels_ << std::endl;

	static unsigned int height_sampling_density = atoi(argv[8]);
	std::cout << "height_sampling_density: " << height_sampling_density<< std::endl;

	static unsigned int angle_sampling_density = atoi(argv[9]);
	std::cout << "angle_sampling_density: " << angle_sampling_density<< std::endl;

	std::vector<double> heights;
	std::vector<double> radii;
	std::vector<double> noise_levels;     // percentage of object radius (std_dev)
	std::vector<double> outlier_levels;   // percentage of object size (std_dev)
	std::vector<double> occlusion_levels; // percentage of object size (std_dev)

	double j;
	std::stringstream ss_(heights_);
	while (ss_ >> j)
	{
	heights.push_back(j);

	if (ss_.peek() == ',')
		ss_.ignore();
	}

	ss_=std::stringstream(radii_);
	while (ss_ >> j)
	{
	radii.push_back(j);

	if (ss_.peek() == ',')
		ss_.ignore();
	}

	ss_=std::stringstream(noise_levels_);
	while (ss_ >> j)
	{
	noise_levels.push_back(j);

	if (ss_.peek() == ',')
		ss_.ignore();
	}


	ss_=std::stringstream(outlier_levels_);
	while (ss_ >> j)
	{
	outlier_levels.push_back(j);

	if (ss_.peek() == ',')
		ss_.ignore();
	}


	ss_=std::stringstream(occlusion_levels_);
	while (ss_ >> j)
	{
	occlusion_levels.push_back(j);

	if (ss_.peek() == ',')
		ss_.ignore();
	}

	unsigned int total_iterations=iterations*outlier_levels.size()*occlusion_levels.size()*heights.size()*radii.size();
	// generate point clouds with different radius and heights at random poses
	for(unsigned int i=0; i<iterations; ++i)
	{
		for(unsigned int o=0; o<outlier_levels.size();++o)
		{
			for(unsigned int occ=0; occ<occlusion_levels.size();++occ)
			{
			    	for(unsigned int h_=0; h_<heights.size();++h_)
			    	{
					for(unsigned int r_=0; r_<radii.size();++r_)
					{
						double height=heights[h_];
						double radius=radii[r_];
						// Generate cylinder according to parameters
						pcl::PointCloud<pcl::PointXYZ> cloud;
						cloud.header.frame_id="world";

						double angle_step=2.0*M_PI/angle_sampling_density;
						double height_step=fabs(height/height_sampling_density);
						unsigned int angle_sampling_density_final=round((1.0-occlusion_levels[occ])*angle_sampling_density);
						for(unsigned int a=0; a < angle_sampling_density_final; ++a)
						{
							double x,y,z;
							x=cos(angle_step*a)*fabs(radius);
							y=sin(angle_step*a)*fabs(radius);
							for(unsigned int h=0; h < height_sampling_density; ++h)
							{
							    z=(double)height_step*h;
							    pcl::PointXYZ point(x,y,z);
							    cloud.push_back(point);
							}
						}


						// top / down surfaces
						//double plane_area=M_PI*radius*radius;
						unsigned int plane_samples=outlier_levels[o]*sqrt(height_sampling_density*angle_sampling_density)*0.5;

						double plane_min=0.0-radius;
						double plane_step=height_step;
						for(unsigned int x_=0; x_<plane_samples;++x_)
						{
							double x=plane_min+x_*plane_step;
							for(unsigned int y_=0; y_<plane_samples;++y_)
							{
							    double y=plane_min+y_*plane_step;
							    cloud.push_back(pcl::PointXYZ(x,y,0));
							    cloud.push_back(pcl::PointXYZ(x,y,height));
							}
						}

						// Generate random orientation
						Eigen::Vector3d cyl_dir=Eigen::Vector3d::UnitZ();
						cyl_dir.normalize();

						Eigen::Vector3d xaxis = Eigen::Vector3d::UnitY().cross(cyl_dir);
						xaxis.normalize();

						Eigen::Vector3d yaxis = cyl_dir.cross(xaxis);
						yaxis.normalize();

						Eigen::Matrix3d rot;
						rot(0,0) = xaxis(0);
						rot(1,0) = yaxis(0);
						rot(2,0) = cyl_dir(0);

						rot(0,1) = xaxis(1);
						rot(1,1) = yaxis(1);
						rot(2,1) = cyl_dir(1);

						rot(0,2) = xaxis(2);
						rot(1,2) = yaxis(2);
						rot(2,2) = cyl_dir(2);

						Eigen::Vector3d cyl_pos(0,0,0);
						Eigen::Matrix4d transf;
						transf.block(0,0,3,3)=rot;
						transf.block(0,3,3,1)=cyl_pos;

						//Transform point cloud
						pcl::transformPointCloud (cloud,cloud,transf);

						unsigned int index=r_+h_*radii.size()+occ*radii.size()*heights.size()+o*radii.size()*heights.size()*occlusion_levels.size()+i*radii.size()*heights.size()*occlusion_levels.size()*outlier_levels.size();

						std::cout << index << " of " << total_iterations << std::endl;

						std::stringstream ss;
						ss << ground_truth_dir << "ground_truth_" << std::setfill('0') << std::setw(6) << index << ".txt";
					
						std::string ground_truth_file=ss.str();
						Cylinder ground_truth(radius,height,cyl_dir,cyl_pos,ground_truth_file);

						// Then corrupt with diferent levels of noise
						for(unsigned int n=0; n<noise_levels.size(); ++n)
						{
							pcl::PointCloud<pcl::PointXYZ> noisy_cloud(cloud);

							std::normal_distribution<> d{0,noise_levels[n]*radii[r_]};
							for(unsigned int p=0; p<noisy_cloud.size();++p)
							{
								noisy_cloud.points[p].x+=d(gen);
								noisy_cloud.points[p].y+=d(gen);
								noisy_cloud.points[p].z+=d(gen);
							}

							std::stringstream ss;
							ss << point_clouds_dir << "cloud_" << std::setfill('0') << std::setw(6) << std::to_string(index) << "_noise_" << std::setfill('0') << std::setw(6) << n << ".pcd";
							pcl::io::savePCDFile(ss.str(), noisy_cloud); // Binary format
						}
					}
				}
			}
		}
	}

	return (0);
}

