
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

namespace point_cloud_color {

/**
 * @brief Provides getters and setters for given field names.
 *
 * Given a set field names, it provides getters and setters for the corresponding fields.
 * It does not handle memory allocation, nor any sanity checks; this has be ensured by the user.
 */
class PointCloudAccessor {
public:
    PointCloudAccessor(sensor_msgs::PointCloud2::Ptr pcl, std::vector<std::string> fieldNames);
    template <class T> T getField(uint32_t pointIndex, uint32_t fieldIndex);
    template <class T> void setField(uint32_t pointIndex, uint32_t fieldIndex, T value);
private:
    sensor_msgs::PointCloud2::Ptr pcl;
    std::vector<std::string> fieldNames;
    std::vector<uint32_t> fieldOffsets;
};

} // namespace point_cloud_color
