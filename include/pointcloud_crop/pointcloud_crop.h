#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<PointT> PointCloudT;

ros::Subscriber point_sub;
ros::Publisher point_pub;

float xyz[6] = {
  0.0,
};

pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_save(new pcl::PointCloud<pcl::PointXYZI>);

bool save_en = true;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& point);
