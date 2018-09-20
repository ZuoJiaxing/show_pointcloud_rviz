#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>


// Downsample, according to the number npoints.
int  randomfilter_XYZI_pts( pcl::PointCloud<pcl::PointXYZI>::Ptr & PtrCloud, int npoints = 8000)
{
    if(PtrCloud->points.size()< npoints)
        return 0;

    ///sample downsample filter, random downsize (takes 0.001s)
    pcl::RandomSample<pcl::PointXYZI> random_sampler;
    // random_sampler.setKeepOrganized(true);// extremly time-consuming
    random_sampler.setInputCloud(PtrCloud);

    uint num_output_points = npoints;
    random_sampler.setSample(num_output_points);
    random_sampler.filter(*PtrCloud);

    return 0;
}


int PassThroughfilter_XYZI(pcl::PointCloud<pcl::PointXYZI>::Ptr & PtrCloud  )
{
    // PassThrough filter
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (PtrCloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.2, 3.5);
    // pass.setFilterFieldName ("y");  //rocky TODO, 应该和激光地图的一致
    // pass.setFilterLimits (-0.5, 0.5);
    //pass.setFilterFieldName ("x");
    //pass.setFilterLimits (-18, +18);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*PtrCloud);
    return 0;
}



main (int argc, char **argv)
{
    ros::init (argc, argv, "readin");

    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("showpointcloud_output", 1);
    sensor_msgs::PointCloud2 output;
    std::string PC_path;
    nhPrivate.param<std::string>("pointcloud_path", PC_path, "/home/rocky/catkin_ws/src/msckf_auto_mapreuse/msckf_data/euroc_mav/V1_03_difficult.pcd");
    int PC_downsamppts;
    nhPrivate.param<int>("pc_downsample_pts", PC_downsamppts, 300000);

	
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

  if (pcl::io::loadPCDFile<pcl::PointXYZI> (PC_path, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file visual1403715590162142976.pcd \n");
    return (-1);
  }

  //Downsample the lidar pointcloud
  randomfilter_XYZI_pts(cloud,PC_downsamppts);
  PassThroughfilter_XYZI(cloud);
  //Convert the cloud to ROS message
  pcl::toROSMsg(*cloud, output);
  output.header.frame_id = "global";

    ros::Rate loop_rate(0.1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
