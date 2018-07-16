/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include "sphere_fitting_hough.h"

int GaussianSphere::iteration;

SphereFittingHough::SphereFittingHough(const GaussianSphere & gaussian_sphere_, unsigned int position_bins_, unsigned int radius_bins_,double min_radius_,double max_radius_, double accumulator_peak_threshold_, bool do_refine_) : 
	gaussian_sphere(gaussian_sphere_),
	radius_bins(radius_bins_),
	position_bins(position_bins_),
        min_radius(min_radius_),
        max_radius(max_radius_),
	r_step((max_radius_-min_radius_)/radius_bins),
	accumulator_peak_threshold(accumulator_peak_threshold_)
{
	// Alocate memory for sphere accumulator
	sphere_accum.resize(radius_bins);
	for(unsigned int r_index=0;r_index<radius_bins;++r_index)
	{
		sphere_accum[r_index].resize(position_bins);
		for(unsigned int x_index=0;x_index<position_bins;++x_index)
		{
			sphere_accum[r_index][x_index].resize(position_bins);
			for(unsigned int y_index=0;y_index<position_bins;++y_index)
			{
				sphere_accum[r_index][x_index][y_index].resize(position_bins);
			}
		}
	}
};

FittingData SphereFittingHough::fit(const PointCloudT::ConstPtr & point_cloud_in_)
{
	// Reset Accumulator;
	for(unsigned int r_index=0;r_index<radius_bins;++r_index)
	{
		for(unsigned int x_index=0;x_index<position_bins;++x_index)
		{
			for(unsigned int y_index=0;y_index<position_bins;++y_index)
			{
				std::fill(sphere_accum[r_index][x_index][y_index].begin(),sphere_accum[r_index][x_index][y_index].end(), 0);
			}
		}
	}

	const std::vector<Eigen::Vector3d> & gaussian_sphere_voting=gaussian_sphere.getGaussianSphere();

	// Get position voting boundaries
	Eigen::Vector4f min_pt,max_pt;
	pcl::getMinMax3D(*point_cloud_in_,min_pt,max_pt);

	double x_position_step=(double)((2.0*max_radius) + (max_pt-min_pt)[0])/position_bins;
	double y_position_step=(double)((2.0*max_radius) + (max_pt-min_pt)[1])/position_bins;
	double z_position_step=(double)((2.0*max_radius) + (max_pt-min_pt)[2])/position_bins;

	double x_position_step_inv=1.0/x_position_step;
	double y_position_step_inv=1.0/y_position_step;
	double z_position_step_inv=1.0/z_position_step;
	
	// Vote
	for(unsigned int i=0; i<gaussian_sphere.gaussian_sphere_points_num; ++i)
	{
		double theta=atan2(gaussian_sphere_voting[i][1],gaussian_sphere_voting[i][0]);
		double phi=acos(gaussian_sphere_voting[i][2]);
		
		double cos_theta=cos(theta);
		double sin_theta=sin(theta);
		double cos_phi=cos(phi);
		double sin_phi=sin(phi);

		for(unsigned int r_index=0; r_index<radius_bins; ++r_index)
		{
			double radius=r_index*r_step+min_radius;
			for(unsigned int s = 0; s < point_cloud_in_->size(); ++s)
			{

				unsigned int cx=floor( (point_cloud_in_->points[s].x-radius*cos_theta*sin_phi - min_pt[0] + max_radius ) * x_position_step_inv);
				unsigned int cy=floor( (point_cloud_in_->points[s].y-radius*sin_theta*sin_phi - min_pt[1] + max_radius ) * y_position_step_inv);
				unsigned int cz=floor( (point_cloud_in_->points[s].z-radius*cos_phi           - min_pt[2] + max_radius ) * z_position_step_inv);
				++sphere_accum[r_index][cx][cy][cz];
			}
		}
	}

	// Get best
	unsigned int best_r_bin=0, best_x_bin=0, best_y_bin=0, best_z_bin=0;
	double most_votes=0.0;
	for(unsigned int r_index=0;r_index<radius_bins;++r_index)
	{
		for(unsigned int x_index=0;x_index<position_bins;++x_index)
		{
			for(unsigned int y_index=0;y_index<position_bins;++y_index)
			{
				for(unsigned int z_index=0;z_index<position_bins;++z_index)
				{
					if(sphere_accum[r_index][x_index][y_index][z_index]>most_votes) 
					{
						best_r_bin=r_index;
						best_x_bin=x_index;
						best_y_bin=y_index;
						best_z_bin=z_index;
						most_votes=sphere_accum[r_index][x_index][y_index][z_index];
					}
				}
			}
		}
	}

	// Recover
	double best_r=best_r_bin*r_step+min_radius;
	double best_x=best_x_bin*x_position_step+(double)min_pt[0] - max_radius;
	double best_y=best_y_bin*y_position_step+(double)min_pt[1] - max_radius;
	double best_z=best_z_bin*z_position_step+(double)min_pt[1] - max_radius;

	Eigen::VectorXf coeffs(4,1);
	coeffs <<  best_x, best_y, best_z, best_r;

	return FittingData(coeffs,1.0);
};

