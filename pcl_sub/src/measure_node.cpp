#include "measure_node.h"

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* request_void)
{
    boost::shared_ptr<KeyMode> request = *static_cast<boost::shared_ptr<KeyMode> *> (request_void);
    std::cout<<"request b4 keyboard: "<<*request<<"\n";
    if (event.getKeySym () == "space" && event.keyDown ())
        *request = NEW_SHOT;
    if (event.getKeySym () == "s" && event.keyDown ())
        *request = SAVE;
    std::cout<<"request after keyboard: "<<*request<<"\n";
};

measureNode::measureNode():
//nh_(ros::this_node::getName()),
request(new KeyMode(NEW_SHOT)),
viewer(new pcl::visualization::PCLVisualizer("ICP demo")),
cloud_in(new PointCloudT),
cloud_icp(new PointCloudT),
coefficients (new pcl::ModelCoefficients),
inliers (new pcl::PointIndices)
{
    capture_counter = 1;
    iterations = 1;
    bckgr_gray_level = 0.0;
    
    // Create a ROS subscriber for the input point cloud
    sub = nh_.subscribe<sensor_msgs::PointCloud2> ("gocator_3200/pcl_output", 1, 
                                                        &measureNode::cloud_cb, this);
    // Publish snap request
    ohSnap = nh_.advertise<std_msgs::Empty>("gocator_3200/snapshot_request",1);
}

measureNode::~measureNode()
{

};

int measureNode::init()
{
    std::string template_filename = "monkey.ply";
    nh_.getParam("template_filename",template_filename);
    nh_.getParam("iterations",iterations);
    //Read PLYFile
    path = ros::package::getPath("gocator_publisher");
    if (pcl::io::loadPLYFile(path + "/model/test/" + template_filename, *cloud_in) < 0)
    {
      PCL_ERROR ("Error loading cloud.\n");
      return (-1);
    }
    return (0);
}

void measureNode::print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void measureNode::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud_icp);
    if (cloud_icp->points.size()!=0)
    {
        *request = WAIT;
        std::cout<<"data received\n";
        initViewer();
        updateViewer();
        sendRequest();
    }
}

void measureNode::initViewer()
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
    
    seg.setInputCloud (cloud_icp);
    
    pcl::console::TicToc pcl_timer;
    pcl_timer.tic();
    seg.segment (*inliers, *coefficients);
    double pencentage = double(inliers->indices.size())/cloud_icp->size();
    std::cout << "Applied "<<std::to_string(iterations) << " ransac iterations in " << pcl_timer.toc () << " ms" << std::endl;
    pcl::ModelCoefficients sphere_coeff;
    sphere_coeff.values.resize(4);
    sphere_coeff.values[0] = coefficients->values[0]; //x
    sphere_coeff.values[1] = coefficients->values[1]; //y
    sphere_coeff.values[2] = coefficients->values[2]; //z
    sphere_coeff.values[3] = coefficients->values[3]; //radius
    
    // Visualization
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180,20,20);
    viewer->addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp");
    viewer->addSphere(sphere_coeff);
    // Adding text descriptions in each viewport
    viewer->addText ("Red: Original point cloud\nWhite: Ransac result", 10, 10, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "ransac_info");
    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "RANSAC iterations = " + ss.str ();
    viewer->addText (iterations_cnt, 10, 50, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
    viewer->addText ("target ball radius: " + std::to_string(coefficients->values[3])+"\n", 10, 70, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "radius");
    viewer->addText ("Pencentage of inliers: " + std::to_string(pencentage) + "\n", 10, 90, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "inliers pencentage");
       // Set background color
    viewer->setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);
       // Set camera position and orientation
    viewer->setCameraPosition (0, 0, 200, 0, 0, 0, 0);
    viewer->addCoordinateSystem(10);
    viewer->setSize (1280, 1024);  // Visualiser window size

    // Register keyboard callback :
    viewer->registerKeyboardCallback (&keyboardEventOccurred, (void*) &request);
}

void measureNode::updateViewer()
{
    while (!viewer->wasStopped())
    {
        viewer->spinOnce ();
        
        // The user pressed "space" :
        if (*request == NEW_SHOT)
        {
            PCL_ERROR ("\nBad result, next sample.\n");
            *request = NEW_SHOT;
        }
    
        if (*request == SAVE)
        {
            std::string file_name = path + "/model/test/"+ std::to_string(capture_counter) +".ply";
            if( pcl::io::savePLYFileASCII (file_name, *cloud_icp) != 0)
            {
                std::cout<<"failed to  save "<<file_name<<"\n";
            }else{
                std::cout<<file_name<<" saved successflly!\n";
                capture_counter++;
            }

            results.open(path + "results/test.txt", std::ios_base::app);
            results << "sphere is positioned at: (in frame)\n";
            results << "  Translation vector :\n";
            results << coefficients->values[0]<< ", " << coefficients->values[1] << ", " << coefficients->values[2]<<"\n";
            results.close();
            std::cout<<"matching finished,exiting...\n";
            *request = NEW_SHOT;
        }
    }
}

void measureNode::sendRequest()
{
    if (*request != WAIT)
    {
        std::cout<<"sending request\n";
        ohSnap.publish(myMsg);
        viewer->close(); 
        *request = WAIT;
    }
}