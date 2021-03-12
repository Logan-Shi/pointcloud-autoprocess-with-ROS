#include "measure_node.h"

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* request_void)
{
    boost::shared_ptr<int> request = *static_cast<boost::shared_ptr<int> *> (request_void);
    std::cout<<"request b4 keyboard: "<<*request<<"\n";
    if (event.getKeySym () == "space" && event.keyDown ())
        *request = 1;
    std::cout<<"request after keyboard: "<<*request<<"\n";
};

measureNode::measureNode():
//nh_(ros::this_node::getName()),
is_send_request(new int),
viewer(new pcl::visualization::PCLVisualizer("ICP demo")),
cloud_in(new PointCloudT),
cloud_icp(new PointCloudT)
{
    iterations = 1;
    bckgr_gray_level = 0.0;
    *is_send_request = 1;
    
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
    std::string path = ros::package::getPath("gocator_publisher");
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
        *is_send_request = 0;
        std::cout<<"data received\n";
    }
    
    // The Iterative Closest Point algorithm
    pcl_timer.tic ();
    icp.setMaximumIterations (iterations);
    icp.setInputSource (cloud_icp);
    icp.setInputTarget (cloud_in);

    initViewer();
    updateViewer();
}

void measureNode::initViewer()
{
    txt_gray_lvl = 1.0 - bckgr_gray_level;
    
    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                               (int) 255 * txt_gray_lvl);
    viewer->addPointCloud (cloud_in, cloud_in_color_h, "cloud_in");
    
    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
    viewer->addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp");
    
    // Adding text descriptions in each viewport
    viewer->addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2");

    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str ();
    viewer->addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
    
    // Set background color
    viewer->setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);
    
    // Register keyboard callback :
    viewer->registerKeyboardCallback (&keyboardEventOccurred, (void*) &is_send_request);
}

void measureNode::updateViewer()
{
    // The Iterative Closest Point algorithm
    pcl_timer.tic ();
    std::cout<<"aligning...\n";
    icp.align (*cloud_icp);
    std::cout << "Applied "<<iterations<<" ICP iteration(s) in " << pcl_timer.toc () << " ms" << std::endl;
    if (icp.hasConverged ())
    {
        // //printf ("\033[11A");  // Go up 11 lines in terminal output.
        printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
        // std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
        print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose
        // ss.str("");
        // ss << iterations;
        // std::string iterations_cnt = "ICP iterations = " + ss.str ();
        // viewer->updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
    }
    while (!viewer->wasStopped())
    {
        viewer->spinOnce ();
        
        pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
        viewer->updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
    }
}

void measureNode::sendRequest()
{
    if (*is_send_request)
    {
        std::cout<<"sending request\n";
        ohSnap.publish(myMsg);   
        *is_send_request = 0;
    }
}