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
viewer(new pcl::visualization::PCLVisualizer("Ransac demo"))
{
    capture_counter = 1;
    iterations = 1;
    
    // Create a ROS subscriber for the input point cloud
    sub = nh_.subscribe<sensor_msgs::PointCloud2> ("gocator_3200/pcl_output", 1, 
                                                        &measureNode::cloud_cb, this);
    // Publish snap request
    ohSnap = nh_.advertise<std_msgs::Empty>("gocator_3200/snapshot_request",1);
    moveIt = nh_.advertise<std_msgs::Empty>("gocator_3200/move_request",1);

    int measure_type;
    nh_.getParam("iterations",iterations);
    nh_.getParam("measure_type",measure_type); runMode = (RunMode)measure_type;
    nh_.getParam("diameter",diameter);
    nh_.getParam("buffer",buffer);
    nh_.getParam("plane_threshold",plane_threshold);
    nh_.getParam("crop_size",crop_size);
    nh_.getParam("threshold",threshold);
    nh_.getParam("radius_search_small",radius_search_small);
    nh_.getParam("radius_search_large",radius_search_large);
    nh_.getParam("angle_threshold",angle_threshold);

    file_path = ros::package::getPath("gocator_publisher");
    
    if (runMode == TARGET_BALL)
    {
        results.open(file_path + "/results/test.txt", std::ios_base::app);
        results << "\nmeasuring TARGET_BALL: \n";
        results.close();
        // measure_target_ball();
    }
    
    if (runMode == WORKPIECE)
    {
        results.open(file_path + "/results/test.txt", std::ios_base::app);
        results << "\nmeasuring WORKPIECE at: \n";
        results.close();
        // measure_workpiece();
    }
}

measureNode::~measureNode()
{

};

void measureNode::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    std::cout<<"cloud received\n";
    PointCloudT::Ptr cloud_in (new PointCloudT);
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
            // results.open(file_path + "/results/test.txt", std::ios_base::app);
            // results << "measuring TARGET_BALL at: "<<cloud_in->header.stamp<<"\n";
            // results.close();
            measure_target_ball(cloud_in);
        }
        
        if (runMode == WORKPIECE)
        {
            // results.open(file_path + "/results/test.txt", std::ios_base::app);
            // results << "measuring WORKPIECE at: "<<cloud_in->header.stamp<<"\n";
            // results.close();
            measure_workpiece(cloud_in);
        }
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        // viewer->removeAllCoordinateSystems();
        viewer->removeText3D();
        // sendRequest();
    }
}

void measureNode::measure_target_ball(const PointCloudT::Ptr cloud_in)
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
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.segment (*inliers, *coefficients);
    double percentage = double(inliers->indices.size())/cloud_in->size();
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
    // viewer->registerKeyboardCallback (&keyboardEventOccurred, (void*) &request);

    results.open(file_path + "/results/test.txt", std::ios_base::app);
    results << "["<<coefficients->values[0]<< ", " << coefficients->values[1] << ", " << coefficients->values[2]<<"]\n";
    results.close();
    moveIt.publish(myMsg);
}

void measureNode::measure_workpiece(const PointCloudT::Ptr cloud_in)
{
    PointCloudT::Ptr cloud_p (new PointCloudT);
    calc_plane(cloud_in, cloud_p, plane_threshold, crop_size,iterations);
  
    PointCloudT::Ptr cloud_boundary (new PointCloudT);
    calc_boundary(cloud_p, cloud_boundary, radius_search_small,radius_search_large,angle_threshold);
  
    double percentage = 0;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    percentage = calc_circle(cloud_boundary, coefficients, percentage, 4, buffer, threshold,iterations);
    std::cout<<"percentage: "<<percentage<<"\n";
    percentage = calc_circle(cloud_boundary, coefficients, percentage, 9, buffer, threshold,iterations);
    std::cout<<"percentage: "<<percentage<<"\n";
    percentage = calc_circle(cloud_boundary, coefficients, percentage, 26, buffer, threshold,iterations);
    std::cout<<"percentage: "<<percentage<<"\n";

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
    viewer->addCoordinateSystem(10,coefficients->values[0],coefficients->values[1],coefficients->values[2]);
  
    // Adding text descriptions in each viewport
    viewer->addText ("White: Original point cloud\nRed: Ransac Plane result\nGreen: Boundary result", 10, 10, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "ransac_info");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_boundary");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5,"cloud_in");

    // Set camera position and orientation
    viewer->setCameraPosition (0, 0, 200, 0, 0, 0, 0);
    viewer->addCoordinateSystem(10);
    viewer->setSize (1280, 1024);  // Visualiser window size

    // Register keyboard callback :
    // viewer->registerKeyboardCallback (&keyboardEventOccurred, (void*) &request);

    bool is_quit = true;
    results.open(file_path + "/results/test.txt", std::ios_base::app);
    results << "["<<coefficients->values[0]<< ", " << coefficients->values[1] << ", " << coefficients->values[2]<<"]\n";
    results.close();
    // std::string file_name = file_path + "/model/test/"+ std::to_string(capture_counter) +".ply";
    // if( pcl::io::savePLYFileASCII (file_name, *cloud_in) != 0)
    // {
    //     std::cout<<"failed to  save "<<file_name<<"\n";
    // }else{
    //     std::cout<<file_name<<" saved successfully!\n";
    //     capture_counter++;
    // }
    moveIt.publish(myMsg);

    while (!is_quit)
    {
        viewer->spinOnce ();
        
        // The user pressed "space" :
        if (*request == NEW_SHOT)
        {
            ROS_INFO ("\nnext sample.\n");
            sendRequest();
            moveIt.publish(myMsg);
            is_quit = true;
            // std::string file_name = file_path + "/model/test/"+ std::to_string(capture_counter) +".ply";
            // if( pcl::io::savePLYFileASCII (file_name, *cloud_in) != 0)
            // {
            //     std::cout<<"failed to  save "<<file_name<<"\n";
            // }else{
            //     std::cout<<file_name<<" saved successflly!\n";
            //     capture_counter++;
            // }
        }
    
        if (*request == SAVE)
        {
            std::string file_name = file_path + "/model/test/"+ std::to_string(capture_counter) +".ply";
            if( pcl::io::savePLYFileASCII (file_name, *cloud_in) != 0)
            {
                std::cout<<"failed to  save "<<file_name<<"\n";
            }else{
                std::cout<<file_name<<" saved successfully!\n";
                capture_counter++;
            }

            // results.open(file_path + "/results/test.txt", std::ios_base::app);
            // results << coefficients->values[0]<< ", " << coefficients->values[1] << ", " << coefficients->values[2]<<"\n";
            // results.close();
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
        ohSnap.publish(myMsg);
        *request = WAIT;
    }
}