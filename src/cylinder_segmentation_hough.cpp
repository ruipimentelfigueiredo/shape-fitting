#include "cylinder_segmentation_hough.h"


int GaussianSphere::iteration;

CylinderSegmentationHough::CylinderSegmentationHough(const GaussianSphere & gaussian_sphere_, unsigned int angle_bins_,unsigned int radius_bins_, unsigned int position_bins_, double min_radius_, double max_radius_, double accumulator_peak_threshold_, unsigned int mode_, bool do_refine_) : 
	CylinderSegmentation(min_radius_,max_radius_,do_refine_),
	gaussian_sphere(gaussian_sphere_),
	angle_bins(angle_bins_),
	angle_step(2*M_PI/angle_bins),
	radius_bins(radius_bins_),
	position_bins(position_bins_),
	r_step((max_radius-min_radius)/radius_bins),
	accumulator_peak_threshold(accumulator_peak_threshold_),
	mode(mode_)
{
	// Alocate memory for direction accumulator
	cyl_direction_accum.resize(gaussian_sphere.gaussian_sphere_points_num);

	// Alocate memory for circle accumulator
	cyl_circ_accum.resize(position_bins);
	for(unsigned int i=0;i<position_bins;++i)
	{
		cyl_circ_accum[i].resize(position_bins);
		for(unsigned int j=0;j<position_bins;++j)
		{
			cyl_circ_accum[i][j].resize(radius_bins);
		}
	}

	ne.setKSearch (50);
	ne.setSearchMethod (tree);
};




Eigen::Vector3d CylinderSegmentationHough::findCylinderDirection(const NormalCloudT::ConstPtr & cloud_normals, const PointCloudT::ConstPtr & point_cloud_in_)
{


	// Setup the principal curvatures computation
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;

	// Provide the original point cloud (without normals)
	principal_curvatures_estimation.setInputCloud ( point_cloud_in_);

	// Provide the point cloud with normals
	principal_curvatures_estimation.setInputNormals (cloud_normals);

	// Use the same KdTree from the normal estimation
	principal_curvatures_estimation.setSearchMethod (tree);
	principal_curvatures_estimation.setKSearch (50);

	// Actually compute the principal curvatures
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
	principal_curvatures_estimation.compute (*principal_curvatures);


	//3. for each point normal
	//ROS_DEBUG_STREAM(" 3. Step 1");
			
	//ROS_DEBUG_STREAM("  3.1. Reset accumulator");

	std::fill(cyl_direction_accum.begin(),cyl_direction_accum.end(), 0);

	const std::vector<Eigen::Vector3d> & gaussian_sphere_voting=gaussian_sphere.getGaussianSphere();
	//ROS_DEBUG_STREAM("  3.2. Vote");
	for(unsigned int i=0; i<gaussian_sphere_voting.size(); ++i)
	{
		Eigen::Vector3d voting_direction=gaussian_sphere_voting[i];
		for(unsigned int s = 0; s < cloud_normals->size(); ++s)
		{
			double normal_weight=(1.0-fabs(cloud_normals->points[s].getNormalVector3fMap().cast<double>().dot(voting_direction)));
			if(mode==NORMAL)
			{
				//1 - fabs(dot.product)
				cyl_direction_accum[i]+=normal_weight;
			}
			else if(mode==CURVATURE)
			{
				double curvature_weight=(1.0-fabs(Eigen::Vector3d(principal_curvatures->points[s].principal_curvature[0],principal_curvatures->points[s].principal_curvature[1],principal_curvatures->points[s].principal_curvature[2]).dot(voting_direction)));
				//Eigen::Vector3d curvature_dir=cloud_normals->points[s].getNormalVector3fMap().cast<double>().cross(voting_direction);

				cyl_direction_accum[i]+=curvature_weight*principal_curvatures->points[s].pc1;//*10.0;
			}
			else if(mode==HYBRID)
			{
				double curvature_weight=(1.0-fabs(Eigen::Vector3d(principal_curvatures->points[s].principal_curvature[0],principal_curvatures->points[s].principal_curvature[1],principal_curvatures->points[s].principal_curvature[2]).dot(voting_direction)));
				//Eigen::Vector3d curvature_dir=cloud_normals->points[s].getNormalVector3fMap().cast<double>().cross(voting_direction);

				cyl_direction_accum[i]+=normal_weight*curvature_weight*principal_curvatures->points[s].pc1;//*principal_curvatures->points[s].pc1;
			}
		}

	}


	std::vector<Eigen::Vector3d> best_orientations;
	// Get best orientation
	double most_votes=0.0;
	unsigned int best_direction_index=0;
	for (unsigned int i=0; i<gaussian_sphere_voting.size(); ++i)
	{
		if(cyl_direction_accum[i]>most_votes)
		{
			best_direction_index=i;
			most_votes=cyl_direction_accum[i];
		}
	}

	best_orientations.push_back(gaussian_sphere_voting[best_direction_index]);

	// Choose orientation whose votes are a percentage above a given threshold of the best orientation
	cyl_direction_accum[best_direction_index]=0; 	// This is more efficient than having an if condition to verify if we are considering the best pose again
	for (unsigned int i=0; i<gaussian_sphere_voting.size(); ++i)
	{
		if(cyl_direction_accum[i]>=accumulator_peak_threshold*most_votes)
		{
                        best_orientations.push_back(gaussian_sphere_voting[i]);
		}
	}



	// HERE WE SHOULD CLUSTER

	//ROS_DEBUG_STREAM("  3.4. Convert back to continuous");

	////ROS_DEBUG_STREAM("    best votes="<< most_votes<<" best_direction_index="<<best_direction_index);

	//ROS_DEBUG_STREAM("  3.5. Convert to direction vector");
	return gaussian_sphere_voting[best_direction_index];
}

Eigen::Matrix<double,5,1> CylinderSegmentationHough::findCylinderPositionRadius(const PointCloudT::ConstPtr & point_cloud_in_)
{
	// Get position voting boundaries
	Eigen::Vector4f min_pt,max_pt;
	pcl::getMinMax3D(*point_cloud_in_,min_pt,max_pt);
	////ROS_DEBUG_STREAM(" min="<< min_pt <<" max="<<max_pt);
	double u_position_step=(double)(max_pt-min_pt)[0]/position_bins;
	double v_position_step=(double)(max_pt-min_pt)[1]/position_bins;

	//ROS_DEBUG_STREAM("  4.1. Reset accumulator");
	for (unsigned int u_index=0; u_index < position_bins; ++u_index) {
		for (unsigned int v_index=0; v_index < position_bins; ++v_index) {
			std::fill(cyl_circ_accum[u_index][v_index].begin(),cyl_circ_accum[u_index][v_index].end(), 0);
		}
	}
	
	//ROS_DEBUG_STREAM("  4.2. Vote");
	for(unsigned int r=0; r<radius_bins;++r)
	{	
		double current_radius=r_step*r+min_radius;

		for(unsigned int w=0; w<angle_bins;++w)
		{
			double current_angle=w*angle_step;
			for(PointCloudT::const_iterator it = point_cloud_in_->begin(); it != point_cloud_in_->end(); ++it)
			{
				double u=it->x - (double)min_pt[0];
				double v=it->y - (double)min_pt[1] ;
		
				// Get discretized coordinates
				unsigned int u_hough=floor( (current_radius*cos(current_angle)+u)/u_position_step);
				unsigned int v_hough=floor( (current_radius*sin(current_angle)+v)/v_position_step);

				//if(u_hough<0||v_hough<0||u_hough>=position_bins||v_hough>=position_bins)
				if(u_hough>=position_bins)
					continue;//u_hough=position_bins-1;
				if(v_hough>=position_bins)
					continue;//v_hough=position_bins-1;
				//	continue;


				++cyl_circ_accum[u_hough][v_hough][r];
			}
		}
	}

	// Get best
	unsigned int best_u_bin=0, best_v_bin=0, best_r_bin=0;
	double most_votes=0.0;
	for (unsigned int u_index=0; u_index < position_bins; ++u_index) {
		for (unsigned int v_index=0; v_index < position_bins; ++v_index) {
			for (unsigned int r_index=0; r_index < radius_bins; ++r_index) {
				if(cyl_circ_accum[u_index][v_index][r_index]>most_votes) 
				{
					best_u_bin=u_index;
					best_v_bin=v_index;
					best_r_bin=r_index;
					most_votes=cyl_circ_accum[u_index][v_index][r_index];
				}
			}
		}
	}

	// Recover
	double best_u=best_u_bin*u_position_step+(double)min_pt[0];
	double best_v=best_v_bin*v_position_step+(double)min_pt[1];
	double best_r=best_r_bin*r_step+min_radius;
	////ROS_DEBUG_STREAM("    best votes="<< most_votes<<" best_u="<< best_u_bin <<" best_v="<<best_v_bin<<" best_r="<<best_r);
	// Get u v in original frame
	Eigen::Matrix<double,5,1> result;
	result << best_u, best_v, min_pt[2], best_r, ((double)max_pt[2]-(double)min_pt[2]);
	return result;

}

CylinderFitting CylinderSegmentationHough::segment(const PointCloudT::ConstPtr & point_cloud_in_)
{
	//1.  Estimate point normals
	//ROS_DEBUG_STREAM(" 2. Estimate normals");

	ne.setInputCloud (point_cloud_in_);

  	//ne.setRadiusSearch (0.03);

	ne.compute (*cloud_normals);


	//std::cout << "output points.size (): " << principal_curvatures->points.size () << std::endl;

    	Eigen::Vector3d cylinder_direction=findCylinderDirection(cloud_normals,point_cloud_in_);
	
	//ROS_DEBUG_STREAM(" 4. Step 2");

	//Get rotation matrix
	Eigen::Matrix4d R2;
	R2=Eigen::Matrix4d::Identity();

   	Eigen::Vector3d up = Eigen::Vector3d::UnitZ();

	if(up.dot(cylinder_direction)<0)
	{
		cylinder_direction=-cylinder_direction;
	}
       	//Eigen::Vector3d rot_axis = up.cross(cylinder_direction);
	Eigen::Vector3d rot_axis = cylinder_direction.cross(up);

	rot_axis.normalize();
	if(std::isnan(rot_axis[0])||std::isnan(rot_axis[1])||std::isnan(rot_axis[2]))
	{
		R2=Eigen::Matrix4d::Identity();
	}
	else
	{
		Eigen::Matrix3d aux;
		aux=Eigen::AngleAxisd(acos(cylinder_direction.dot(up)),rot_axis);
		R2.block(0,0,3,3)=aux;
	}

	//R2.block(0,3,3,1)=min_pt.block(0,0,3,1);
    	//std::cout << R2 << std::endl;
	//std::cout << "result rot:" << std::endl << R2.block(0,0,3,3)*cylinder_direction << std::endl;
  	// HERE FILTER POINTS THAT HAVE NORMAL NOT PERPENDICULAR TO CILINDER DIRECTION (CHECK CROSS PRODUCT)

	// Extract the cylinder inliers from the input cloud
	double thresh_=fabs(cos(angle_step));
	pcl::PointIndices::Ptr  inliers_cylinder (new pcl::PointIndices);
	for (unsigned i=0; i < cloud_normals->points.size(); ++i) 
	{
		double dot_product=cloud_normals->points[i].getNormalVector3fMap ().cast<double>().dot(cylinder_direction);

		if(fabs(dot_product)<thresh_)
		{
			inliers_cylinder->indices.push_back(i);
		}
	}

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (point_cloud_in_);
	extract.setIndices (inliers_cylinder);
	extract.setNegative (false);
	extract.filter (*transformed_cloud);

	// Executing the transformation that aligns the cylinder rotation axis with z_up)
	pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, R2);

	Eigen::Matrix<double,5,1> position_and_radius=findCylinderPositionRadius(transformed_cloud);
	double radius=position_and_radius[3];
	double height=position_and_radius[4];
	// Convert back to original coordinates
	Eigen::Vector3d cylinder_position=R2.block(0,0,3,3).transpose()*position_and_radius.block(0,0,3,1);


	Eigen::VectorXf coeffs(7,1);
	coeffs << 
		cylinder_position[0]+0.5*height*cylinder_direction[0],
		cylinder_position[1]+0.5*height*cylinder_direction[1],
		cylinder_position[2]+0.5*height*cylinder_direction[2],
		cylinder_direction[0],
		cylinder_direction[1],
		cylinder_direction[2],
		radius;
		//height;

	// Create the filtering object
	PointCloudT::Ptr cloud_projected(new PointCloudT);
	pcl::SampleConsensusModelCylinder<PointT, NormalT>::Ptr dit (new pcl::SampleConsensusModelCylinder<PointT,NormalT> (point_cloud_in_)); 
    	dit->setInputNormals(cloud_normals); 

	std::vector<int> inliers; 
	dit -> selectWithinDistance (coeffs, 0.01, inliers); 
	pcl::copyPointCloud<PointT>(*point_cloud_in_, inliers, *cloud_projected); 

	double inlier_ratio_=((double)cloud_projected->size()/(double)point_cloud_in_->size());

	// Refine height
	//Eigen::Vector4d min_pt,max_pt;
	//pcl::getMinMax3D(*cloud_projected,min_pt,max_pt);
	//height=max_pt[2]-min_pt[2];

	// Redefine cylinder position (base);
	//Eigen::Vector4d refined_cylinder_position=R2.transpose()*Eigen::Vector4d(best_u,best_v,min_pt[2],0.0);
    	//coefficients_cylinder->values[0]=refined_cylinder_position[0];
    	//coefficients_cylinder->values[1]=refined_cylinder_position[1];
    	//coefficients_cylinder->values[2]=refined_cylinder_position[2];

	//std::cout << "height:" << height << std::endl;


	Eigen::VectorXf final_coeffs(8,1);
	final_coeffs << coeffs,
			height;

	CylinderFitting cylinder_fitting(final_coeffs,inlier_ratio_);

	return cylinder_fitting;
}

