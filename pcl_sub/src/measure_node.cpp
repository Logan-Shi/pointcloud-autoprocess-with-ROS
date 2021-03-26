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
cloud_icp(new PointCloudT),
coefficients (new pcl::ModelCoefficients)
{
    capture_counter = 1;
    iterations = 1;
    
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
    nh_.getParam("iterations",iterations);
    path = ros::package::getPath("gocator_publisher");
    return (0);
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
        // sendRequest();
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
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    seg.segment (*inliers, *coefficients);
    // double pencentage = double(inliers->indices.size())/cloud_icp->size();
    // std::cout << "Applied "<<std::to_string(iterations) << " ransac iterations in " << pcl_timer.toc () << " ms" << std::endl;
    pcl::ModelCoefficients sphere_coeff;
    sphere_coeff.values.resize(4);
    sphere_coeff.values[0] = coefficients->values[0]; //x
    sphere_coeff.values[1] = coefficients->values[1]; //y
    sphere_coeff.values[2] = coefficients->values[2]; //z
    sphere_coeff.values[3] = coefficients->values[3]; //radius
    
    // Visualization
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180,20,20);
    viewer->addPointCloud (cloud_icp, cloud_icp_color_h);
    viewer->addSphere(sphere_coeff);

    // Set camera position and orientation
    viewer->setCameraPosition (0, 0, 200, 0, 0, 0, 0);
    viewer->addCoordinateSystem(10);
    viewer->setSize (1280, 1024);  // Visualiser window size

    // Register keyboard callback :
    viewer->registerKeyboardCallback (&keyboardEventOccurred, (void*) &request);
}

void measureNode::updateViewer()
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
            if( pcl::io::savePLYFileASCII (file_name, *cloud_icp) != 0)
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