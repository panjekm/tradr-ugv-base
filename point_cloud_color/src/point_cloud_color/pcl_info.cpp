
#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include "point_cloud_color/point_cloud_accessor.h"

namespace point_cloud_color {
class PclInfo {
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    std::string pclPath;
    sensor_msgs::PointCloud2::Ptr cloudPtr;
public:
    PclInfo();
//    virtual ~PclInfo();
};

PclInfo::PclInfo() : nh(), pnh("~"), pclPath(), cloudPtr(new sensor_msgs::PointCloud2) {
    pnh.param("pcl_path", pclPath, pclPath);
    ROS_INFO("PCL path: %s.", pclPath.c_str());
    pcl::io::loadPCDFile(pclPath, *cloudPtr);
    std::vector<std::string> fieldNames;
    fieldNames.push_back("x");
    fieldNames.push_back("y");
    fieldNames.push_back("z");
    PointCloudAccessor cloudIter(cloudPtr, fieldNames);
    uint32_t numPoints = cloudPtr->width * cloudPtr->height;
    int debug = 0;
    for (uint32_t iPoint = 0; iPoint < numPoints && debug++ < 10; iPoint++) {
        for (uint32_t iField = 0; iField < fieldNames.size(); iField++) {
            ROS_INFO("Point %i, field %i: %.2f.", iPoint, iField, cloudIter.getField<float>(iPoint, iField));
        }
    }
}

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcl_info");
    point_cloud_color::PclInfo node;
    ros::spin();
}
