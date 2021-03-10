#include "measure_node.h"

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* request_void)
{
    boost::shared_ptr<int> request = *static_cast<boost::shared_ptr<int> *> (request_void);
    if (event.getKeySym () == "space" && event.keyDown ())
        *request = 1;
};

measureNode::measureNode():
//nh_(ros::this_node::getName()),
is_send_request(new int),
cloud_in(new PointCloudT),
cloud_icp(new PointCloudT),
cloud_tr(new PointCloudT)
{
    *is_send_request = 1;
    
    // Create a ROS subscriber for the input point cloud
    sub = nh_.subscribe<sensor_msgs::PointCloud2> ("gocator_3100/pcl_output", 1, 
                                                        &measureNode::cloud_cb, this);
    // Publish snap request
    ohSnap = nh_.advertise<std_msgs::Empty>("gocator_3100/snapshot_request",1);
}

measureNode::~measureNode()
{

};

int measureNode::init(const std::string& file_name)
{
    //Read PLYFile
    std::string path = ros::package::getPath("gocator_publisher");
    if (pcl::io::loadPLYFile(path + "/model/" + file_name + ".ply", *cloud_in) < 0)
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
    *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use
    
    // The Iterative Closest Point algorithm
    pcl_timer.tic ();
    icp.setMaximumIterations (iterations);
    icp.setInputSource (cloud_icp);
    icp.setInputTarget (cloud_in);

    initViewer();
}

void measureNode::initViewer()
{
    // Create two vertically separated viewports
    int v1 (0);
    int v2 (1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    txt_gray_lvl = 1.0 - bckgr_gray_level;
    
    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                               (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);
    
    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
    viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);
    
    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
    viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);
    
    // Adding text descriptions in each viewport
    viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);
    //viewer.addCube(box_min(0),box_max(0),box_min(1),box_max(1),box_min(2),box_max(2),1.0,0.0,0.0,"crop box",v1);
    //viewer.addCoordinateSystem(1.0,0,0,0,"cloud_in",v1);

    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str ();
    viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
    
    // Set background color
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
    
    // Set camera position and orientation
    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);  // Visualiser window size
    
    // Register keyboard callback :
    viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) &(this->is_send_request));
}

void measureNode::updateViewer()
{
    while (!*is_send_request)
    {
        viewer.spinOnce ();
        
        // The Iterative Closest Point algorithm
        pcl_timer.tic ();
        icp.align (*cloud_icp);
        // std::cout << "Applied 1 ICP iteration in " << pcl_timer.toc () << " ms" << std::endl;
        if (icp.hasConverged ())
        {
            // //printf ("\033[11A");  // Go up 11 lines in terminal output.
            printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
            std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
            transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
            print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose
            ss.str("");
            ss << iterations;
            std::string iterations_cnt = "ICP iterations = " + ss.str ();
            viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
            pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
            viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
        }
        else
        {
            PCL_ERROR ("\nICP has not converged.\n");
        }
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