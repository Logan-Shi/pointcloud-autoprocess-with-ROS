#include "measure_node.h"

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* request_void)
{
    boost::shared_ptr<KeyMode> request = *static_cast<boost::shared_ptr<KeyMode> *> (request_void);
    // std::cout<<"request b4 keyboard: "<<*request<<"\n";
    if (event.getKeySym () == "space" && event.keyDown ())
        *request = NEW_SHOT;
    if (event.getKeySym () == "s" && event.keyDown ())
        *request = SAVE;
    // std::cout<<"request after keyboard: "<<*request<<"\n";
};

measureNode::measureNode():
request(new KeyMode(NEW_SHOT)),
viewer(new pcl::visualization::PCLVisualizer("Ransac demo")),
cloud_in(new PointCloudT),
coefficients (new pcl::ModelCoefficients)
{
    capture_counter = 1;
    iterations = 1;
    
    // Create a ROS subscriber for the input point cloud
    sub = nh_.subscribe<sensor_msgs::PointCloud2> ("gocator_3200/pcl_output", 1, 
                                                        &measureNode::cloud_cb, this);
    // Publish snap request
    ohSnap = nh_.advertise<std_msgs::Empty>("gocator_3200/snapshot_request",1);

    int measure_type;
    nh_.getParam("iterations",iterations);
    nh_.getParam("measure_type",measure_type); runMode = (RunMode)measure_type;
    path = ros::package::getPath("gocator_publisher");
}

measureNode::~measureNode()
{

};

void measureNode::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud_in);
    if (cloud_in->points.size()!=0)
    {
        *request = WAIT;
        std::cout<<"data received\n";
        if (runMode == TARGET_BALL)
        {
            results.open(path + "/results/test.txt", std::ios_base::app);
            results << "measuring TARGET_BALL at: "<<cloud_in->header.stamp<<"\n";
            results.close();
            measure_target_ball();
        }
        
        if (runMode == WORKPIECE)
        {
            results.open(path + "/results/test.txt", std::ios_base::app);
            results << "measuring WORKPIECE at: "<<cloud_in->header.stamp<<"\n";
            results.close();
            measure_workpiece();
        }
        checkResult();
        // sendRequest();
    }
}

void measureNode::measure_target_ball()
{
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_SPHERE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.001);
    seg.setRadiusLimits(14, 16);
    seg.setMaxIterations (iterations);
    
    seg.setInputCloud (cloud_in);
    
    pcl_timer.tic();
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    seg.segment (*inliers, *coefficients);
    // double pencentage = double(inliers->indices.size())/cloud_in->size();
    std::cout << "Applied "<<std::to_string(iterations) << " ransac iterations in " << pcl_timer.toc () << " ms" << std::endl;
    pcl::ModelCoefficients sphere_coeff;
    sphere_coeff.values.resize(4);
    sphere_coeff.values[0] = coefficients->values[0]; //x
    sphere_coeff.values[1] = coefficients->values[1]; //y
    sphere_coeff.values[2] = coefficients->values[2]; //z
    sphere_coeff.values[3] = coefficients->values[3]; //radius
    
    // Visualization
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, 180,20,20);
    viewer->addPointCloud (cloud_in, cloud_in_color_h);
    viewer->addSphere(sphere_coeff);

    // Set camera position and orientation
    viewer->setCameraPosition (0, 0, 200, 0, 0, 0, 0);
    viewer->addCoordinateSystem(10);
    viewer->setSize (1280, 1024);  // Visualiser window size

    // Register keyboard callback :
    viewer->registerKeyboardCallback (&keyboardEventOccurred, (void*) &request);
}

void measureNode::measure_workpiece()
{
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
    seg.setInputCloud (cloud_in);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    
    pcl_timer.tic ();
    seg.segment (*inliers, *coefficients_plane);
    if (inliers->indices.size() == 0 )
    {
      PCL_ERROR("\n can not extract plane from given data. \n");
    }
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(false);
    PointCloudT::Ptr cloud_p (new PointCloudT);  // Plane point cloud
    extract.filter(*cloud_p);
    std::cout << "Applied "<<std::to_string(iterations) << " plane ransac iterations in " << pcl_timer.toc () << " ms" << std::endl;
    std::cout << "Plane size:  "<<std::to_string(inliers->indices.size()) <<"" << std::endl;
  
    double pencentage = double(inliers->indices.size())/cloud_in->size();
  
    // Boundary
    pcl::PointCloud<pcl::Boundary> boundaries; 
    pcl::BoundaryEstimation<PointT, PointNT, pcl::Boundary> boundEst; 
    pcl::NormalEstimation<PointT, PointNT> normEst; 
    PointCloudNT::Ptr normals(new PointCloudNT); 
    PointCloudT::Ptr cloud_boundary (new PointCloudT); 
    normEst.setInputCloud(cloud_p); 
    normEst.setRadiusSearch(0.5); 
    pcl_timer.tic();
    normEst.compute(*normals); 
    std::cout << "Calculated normals in " << pcl_timer.toc () << " ms" << std::endl;
   
    boundEst.setInputCloud(cloud_p); 
    boundEst.setInputNormals(normals); 
    boundEst.setRadiusSearch(1.5); 
    boundEst.setAngleThreshold(M_PI/4); 
    boundEst.setSearchMethod(pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>)); 
    pcl_timer.tic();
    boundEst.compute(boundaries); 
    std::cout << "Calculated boundaries in " << pcl_timer.toc () << " ms" << std::endl;
   
    for(int i = 0; i < cloud_p->points.size(); i++) 
    {
      if(boundaries[i].boundary_point > 0) 
      { 
        cloud_boundary->push_back(cloud_p->points[i]); 
      }
    }
  
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg_circle;
    // Optional
    seg_circle.setOptimizeCoefficients (true);
    // Mandatory
    seg_circle.setModelType (pcl::SACMODEL_CIRCLE3D);
    seg_circle.setMethodType (pcl::SAC_RANSAC);
    seg_circle.setDistanceThreshold (0.01);
    seg_circle.setRadiusLimits(1, 15);
    seg_circle.setMaxIterations (iterations);
  
    pcl::ExtractIndices<PointT> extract_circle;
    seg_circle.setInputCloud (cloud_boundary);
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_circle (new pcl::PointIndices);
    
    pcl_timer.tic ();
    seg_circle.segment (*inliers_circle, *coefficients);

    std::cout << "Applied "<<std::to_string(iterations) << " circle ransac iterations in " << pcl_timer.toc () << " ms" << std::endl;
  
    float txt_gray_lvl = 1.0;
  
    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, 255,255,255);
    viewer->addPointCloud (cloud_in, cloud_in_color_h, "cloud_in");
  
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_p_color_h (cloud_p, 180,20,20);
    viewer->addPointCloud (cloud_p, cloud_p_color_h, "cloud_p");
  
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_boundary_color_h (cloud_boundary, 20,180,20);
    viewer->addPointCloud (cloud_boundary, cloud_boundary_color_h, "cloud_boundary");
  
    pcl::ModelCoefficients cylinder_coeff;
    double cylinder_length = 15;
    cylinder_coeff.values.resize (7);    // We need 7 values
    cylinder_coeff.values[0] = coefficients->values[0];
    cylinder_coeff.values[1] = coefficients->values[1];
    cylinder_coeff.values[2] = coefficients->values[2];
   
    cylinder_coeff.values[3] = cylinder_length*coefficients->values[4];
    cylinder_coeff.values[4] = cylinder_length*coefficients->values[5];
    cylinder_coeff.values[5] = cylinder_length*coefficients->values[6];
  
    cylinder_coeff.values[6] = coefficients->values[3];
   
    viewer->addCylinder (cylinder_coeff);
    // viewer->addCoordinateSystem(10,coefficients->values[0],coefficients->values[1],coefficients->values[2]);
  
    // Adding text descriptions in each viewport
    viewer->addText ("White: Original point cloud\nRed: Ransac Plane result\nGreen: Boundary result", 10, 10, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "ransac_info");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_boundary");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5,"cloud_in");

    // Set camera position and orientation
    viewer->setCameraPosition (0, 0, 200, 0, 0, 0, 0);
    viewer->addCoordinateSystem(10);
    viewer->setSize (1280, 1024);  // Visualiser window size

    // Register keyboard callback :
    viewer->registerKeyboardCallback (&keyboardEventOccurred, (void*) &request);
}

void measureNode::checkResult()
{
    bool is_quit = false;
    while (!is_quit)
    {
        viewer->spinOnce ();
        
        // The user pressed "space" :
        if (*request == NEW_SHOT)
        {
            ROS_INFO ("\nnext sample.\n");
            sendRequest();
            is_quit = true;
        }
    
        if (*request == SAVE)
        {
            std::string file_name = path + "/model/test/"+ std::to_string(capture_counter) +".ply";
            if( pcl::io::savePLYFileASCII (file_name, *cloud_in) != 0)
            {
                std::cout<<"failed to  save "<<file_name<<"\n";
            }else{
                std::cout<<file_name<<" saved successflly!\n";
                capture_counter++;
            }

            results.open(path + "/results/test.txt", std::ios_base::app);
            // results << "sphere is positioned at: (in frame)\n";
            // results << "  Translation vector :\n";
            results << coefficients->values[0]<< ", " << coefficients->values[1] << ", " << coefficients->values[2]<<"\n";
            results.close();
            // std::cout<<path + "/results/test.txt saved successflly!\n";
            *request = WAIT;
        }
    }
}

void measureNode::sendRequest()
{
    if (*request != WAIT)
    {
        std::cout<<("sending request\n");
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        // viewer->removeAllCoordinateSystems();
        viewer->removeText3D();
        ohSnap.publish(myMsg);
        *request = WAIT;
    }
}