#include <workpiece_measure.h>

double calc_circle(const PointCloudT::Ptr cloud_boundary, pcl::ModelCoefficients::Ptr coefficients_circle, double diameter, double buffer, double threshold, int iterations)
{
  std::cout << "input size:  "<<std::to_string(cloud_boundary->size()) <<"" << std::endl;
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg_circle;
  // Optional
  seg_circle.setOptimizeCoefficients (true);
  // Mandatory
  seg_circle.setModelType (pcl::SACMODEL_CIRCLE3D);
  seg_circle.setMethodType (pcl::SAC_RANSAC);
  seg_circle.setDistanceThreshold (threshold);
  double radius_min = (diameter-buffer)/2;
  double radius_max = (diameter+buffer)/2;
  std::cout<<"radius_min: "<<radius_min<<"\n";
  std::cout<<"radius_max: "<<radius_max<<"\n";
  seg_circle.setRadiusLimits(radius_min, radius_max);
  seg_circle.setMaxIterations (iterations);

  pcl::ExtractIndices<PointT> extract_circle;
  seg_circle.setInputCloud (cloud_boundary);
  // pcl::ModelCoefficients::Ptr coefficients_circle (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_circle (new pcl::PointIndices);
  
  // pcl_timer.tic ();
  seg_circle.segment (*inliers_circle, *coefficients_circle);
  // if (inliers_circle->indices.size() == 0 )
  // {
  //   PCL_ERROR("\n can not extract circle from given boundaries\n");
  // }
  // extract_circle.setInputCloud(cloud_boundary);
  // extract_circle.setIndices(inliers_circle);
  // extract_circle.setNegative(false);
  // PointCloudT::Ptr cloud_circle (new PointCloudT);  // Plane point cloud
  // extract_circle.filter(*cloud_circle);
  // std::cout << "Applied "<<std::to_string(iterations) << " circle ransac iterations in " << pcl_timer.toc () << " ms" << std::endl;
  std::cout << "Circle size:  "<<std::to_string(inliers_circle->indices.size()) <<"" << std::endl;

  double percentage = double(inliers_circle->indices.size())/cloud_boundary->size();

  std::cout<<"circle percentage: "<<percentage<<"\n";
  std::cout<<"circle radius: "<<coefficients_circle->values[3]<<"\n";
  return percentage;
}

void calc_boundary(const PointCloudT::Ptr cloud_p, PointCloudT::Ptr cloud_boundary, double radius_search_small, double radius_search_large, double angle_threshold)
{
  // Boundary
  pcl::PointCloud<pcl::Boundary> boundaries; 
  pcl::BoundaryEstimation<PointT, PointNT, pcl::Boundary> boundEst; 
  pcl::NormalEstimation<PointT, PointNT> normEst; 
  PointCloudNT::Ptr normals(new PointCloudNT); 
  normEst.setInputCloud(cloud_p); 
  normEst.setRadiusSearch(radius_search_small); 
  // pcl_timer.tic();
  normEst.compute(*normals); 
  // std::cout << "Calculated normals in " << pcl_timer.toc () << " ms" << std::endl;
 
  boundEst.setInputCloud(cloud_p);
  boundEst.setInputNormals(normals);
  boundEst.setRadiusSearch(radius_search_large); 
  boundEst.setAngleThreshold(M_PI/angle_threshold); 
  boundEst.setSearchMethod(pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>)); 
  // pcl_timer.tic();
  boundEst.compute(boundaries); 
  // std::cout << "Calculated boundaries in " << pcl_timer.toc () << " ms" << std::endl;
 
  for(int i = 0; i < cloud_p->points.size(); i++) 
  {
    if(boundaries[i].boundary_point > 0) 
    { 
      cloud_boundary->push_back(cloud_p->points[i]); 
    }
  }
}

double calc_plane(const PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_p,double z_min,double z_max,int iterations)
{
  double percentage = 0;
  PointCloudT::Ptr cloud_filtered (new PointCloudT);
  pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT> ());
  range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT>("z",pcl::ComparisonOps::GT,z_min)));
  range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT>("z",pcl::ComparisonOps::LT,z_max)));
  pcl::ConditionalRemoval<PointT> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(cloud_in);
  condrem.setKeepOrganized(false);
  condrem.filter(*cloud_filtered);

  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  // seg.setRadiusLimits(14, 16);
  seg.setMaxIterations (iterations);

  pcl::ExtractIndices<PointT> extract;
  seg.setInputCloud (cloud_filtered);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  
  // pcl_timer.tic ();
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size() == 0 )
  {
    return percentage;
  }
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(false);
  
  extract.filter(*cloud_p);
  // std::cout << "Applied "<<std::to_string(iterations) << " plane ransac iterations in " << pcl_timer.toc () << " ms" << std::endl;
  std::cout << "Plane size:  "<<std::to_string(inliers->indices.size()) <<"" << std::endl;

  percentage = double(inliers->indices.size())/cloud_filtered->size();
  return percentage;
}