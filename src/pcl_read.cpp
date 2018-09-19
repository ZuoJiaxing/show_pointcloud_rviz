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
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    sensor_msgs::PointCloud2 output;
    ros::Publisher pcl_pub_lidar = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output_lidar", 1);
    sensor_msgs::PointCloud2 output_lidar;
	
	
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_lidar (new pcl::PointCloud<pcl::PointXYZI>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/rocky/catkin_ws/src/msckf_auto_mapreuse/msckf_core/visualpcdmap/v102_pcd/visual1403715590162142976.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file visual1403715590162142976.pcd \n");
    return (-1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/rocky/catkin_ws/src/msckf_auto_mapreuse/msckf_data/euroc_mav/V1_03_difficult.pcd", *cloud_lidar) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file .pcd \n");
    return (-1);
  }

    //Convert the cloud to ROS message
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "global";
    //Downsample the lidar pointcloud
    randomfilter_XYZI_pts(cloud_lidar,300000);
    PassThroughfilter_XYZI(cloud_lidar);

    //Convert the cloud to ROS message
    pcl::toROSMsg(*cloud_lidar, output_lidar);
    output_lidar.header.frame_id = "global";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        pcl_pub_lidar.publish(output_lidar);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
