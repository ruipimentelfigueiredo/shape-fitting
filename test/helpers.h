/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/
#ifndef HELPERS_DATA_H
#define HELPERS_DATA_H
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <ctime>
#include <random>
#include "yaml-cpp/yaml.h"


unsigned int sub_directory_count(const std::string& name)
{
    boost::filesystem::path full_path(name);

    unsigned long file_count = 0;
    unsigned long dir_count = 0;
    unsigned long err_count = 0;

    if ( !boost::filesystem::exists( full_path ) )
    {
        std::cout << "\nNot found: " << full_path.filename() << std::endl;
        return 0;
    }

    if ( boost::filesystem::is_directory( full_path ) )
    {
        boost::filesystem::directory_iterator end_iter;
        for ( boost::filesystem::directory_iterator dir_itr( full_path ); dir_itr != end_iter; ++dir_itr)
        {
            try
            {
                if ( boost::filesystem::is_directory( *dir_itr ) )
                {
                    ++dir_count;
                }
                else
                {
                    ++file_count;
                }
            }
            catch ( const std::exception & ex )
            {
                ++err_count;
            }
        }
        //std::cout << "\n" << file_count << " files\n" << dir_count << " directories\n" << err_count << " errors\n";
    }
    else // must be a file
    {
        std::cout << full_path.string() << "\n";    
    }
    return dir_count;
}

bool createDirectory(std::string & path)
{
    boost::filesystem::path dir(path);
    try
    {
        if (boost::filesystem::create_directories(dir) == 0) {
            std::cout << "Created directory " << path << std::endl;
        } else {
            std::cout << "Did not create directory (probably exists) " << path << std::endl;
        }
    }
    catch (boost::filesystem::filesystem_error &e)
    {
        std::cerr << "Could not create directory " << path << std::endl;
        return false;
    }

    return true;
}

struct path_leaf_string
{
    std::string operator()(const boost::filesystem::directory_entry& entry) const
    {
        return entry.path().leaf().string();
    }
};
 
void readDirectory(const std::string& name, std::vector<std::string>& file_names)
{
	boost::filesystem::path p(name);
	boost::filesystem::directory_iterator start(p);
	boost::filesystem::directory_iterator end;
	std::transform(start, end, std::back_inserter(file_names), path_leaf_string());
	std::sort(file_names.begin(), file_names.end());
}

class Cylinder
{

	public:
		Cylinder(const std::string & ground_truth_file)
		{
			std::fstream myfile(ground_truth_file, std::ios_base::in);
			myfile >> std::setprecision(5) 
			       >> radius 
			       >> height 
			       >> direction[0] 
			       >> direction[1]
			       >> direction[2] 
			       >> position[0] 
			       >> position[1] 
			       >> position[2];

			direction.normalize();
		};

		Cylinder(const double & radius_, const double & height_, const Eigen::Vector3d & direction_, const Eigen::Vector3d & position_, const std::string & ground_truth_file) :
			radius(radius_), 
			height(height_),
			direction(direction_), 
			position(position_)
		{
			direction.normalize();
                        std::ofstream myfile;
                        myfile.open(ground_truth_file);
                        myfile << std::setprecision(5)
                               << radius << " "
                               << height << " " 
                               << direction[0] << " "
                               << direction[1] << " "
                               << direction[2] << " "
                               << position[0] << " "
                               << position[1] << " "
                               << position[2];
		};
       		friend std::ostream &operator<<(std::ostream &os, const Cylinder& obj);


		double radius;
		double height;
		Eigen::Vector3d direction;
		Eigen::Vector3d position;
};


class Sphere
{
	public:
		Sphere(const std::string & ground_truth_file)
		{
			std::fstream myfile(ground_truth_file, std::ios_base::in);
			myfile >> std::setprecision(5) 
			       >> radius 
			       >> position[0] 
			       >> position[1] 
			       >> position[2];
		};

		Sphere(const double & radius_, const Eigen::Vector3d & position_, const std::string & ground_truth_file) :
			radius(radius_), 
			position(position_)
		{
                        std::ofstream myfile;
                        myfile.open(ground_truth_file);
                        myfile << std::setprecision(5)
                               << radius << " "
                               << position[0] << " "
                               << position[1] << " "
                               << position[2];
		};
       		friend std::ostream &operator<<(std::ostream &os, const Sphere& obj);


		double radius;
		Eigen::Vector3d position;
};


std::ostream &operator<<(std::ostream &os, const Cylinder& obj) { 
	return os << obj.radius << " " << obj.height << " " << obj.direction.transpose() << " " << obj.position.transpose() << std::endl;
};

std::ostream &operator<<(std::ostream &os, const Sphere& obj) { 
	return os << obj.radius << " " << obj.position.transpose() << std::endl;
};

class GroundTruth 
{
	public:
		GroundTruth(const std::string & dataset_path_)
		{
			std::vector<std::string> file_names;
			boost::filesystem::path p(dataset_path_);
			boost::filesystem::directory_iterator start(p);
			boost::filesystem::directory_iterator end;
			std::transform(start, end, std::back_inserter(file_names), path_leaf_string());
			std::sort(file_names.begin(), file_names.end());

			for(size_t f=0;f<file_names.size();++f)
			{
				ground_truth.push_back(dataset_path_+file_names[f]);
			}		
		};

		std::vector<Cylinder> ground_truth;
};

class PointClouds 
{
	public:
		PointClouds(const std::string & dataset_path_)
		{
			boost::filesystem::path p(dataset_path_);
			boost::filesystem::directory_iterator start(p);
			boost::filesystem::directory_iterator end;
			//std::transform(start, end, std::back_inserter(file_names), path_leaf_string());
			//std::sort(file_names.begin(), file_names.end());

			//boost::filesystem::path p(dataset_path_);
			//boost::filesystem::directory_iterator start(p);
			//boost::filesystem::directory_iterator end;
			file_names.clear();
			std::transform(start, end, std::back_inserter(aux_file_names), path_leaf_string());
			std::sort(aux_file_names.begin(), aux_file_names.end());

			std::string delimiter = ".";
			std::string token ;
			for(size_t f=0;f<aux_file_names.size();++f)
			{
				try
				{
					token = aux_file_names[f].substr(aux_file_names[f].find(delimiter),aux_file_names[f].length());
				}
				catch ( const std::exception & ex )
				{
					continue;
				}
				if(!token.empty())
					file_names.push_back(aux_file_names[f]);
			}	
		};

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPointCloudRGB(const std::string & file_path)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

			if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (file_path, *cloud) == -1) //* load the file
			{
				std::cout << "Couldn't load point cloud" << std::endl;
				exit(-1);
			}
			return cloud;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(const std::string & file_path)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

			if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_path, *cloud) == -1) //* load the file
			{
				std::cout << "Couldn't load point cloud" << std::endl;
				exit(-1);
			}
			return cloud;
		}

		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > point_clouds;
		std::vector<std::string> file_names;
		std::vector<std::string> aux_file_names;
};


#endif // HELPERS_DATA_H


