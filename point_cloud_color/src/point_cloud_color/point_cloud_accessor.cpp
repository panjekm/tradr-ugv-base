
#include "point_cloud_color/point_cloud_accessor.h"

#include <pcl/common/io.h>

namespace point_cloud_color {

PointCloudAccessor::PointCloudAccessor(sensor_msgs::PointCloud2::Ptr pcl, std::vector<std::string> fieldNames):
        pcl(pcl), fieldNames(fieldNames) {

    fieldOffsets.resize(fieldNames.size(), 0);
    for (size_t i = 0; i < fieldNames.size(); i++) {
        int fieldIndex = pcl::getFieldIndex(*pcl, fieldNames[i]);
        fieldOffsets[i] = pcl->fields[fieldIndex].offset;
    }
}

template <class T> T PointCloudAccessor::getField(uint32_t pointIndex, uint32_t fieldIndex) {
    return *reinterpret_cast<T*>(pcl->data.data() + pointIndex * pcl->point_step + fieldOffsets[fieldIndex]);
}
template <> float PointCloudAccessor::getField(uint32_t pointIndex, uint32_t fieldIndex) {
    return *reinterpret_cast<float*>(pcl->data.data() + pointIndex * pcl->point_step + fieldOffsets[fieldIndex]);
}
template <class T> void PointCloudAccessor::setField(uint32_t pointIndex, uint32_t fieldIndex, T value) {
    *reinterpret_cast<T*>(pcl->data.data() + pointIndex * pcl->point_step + fieldOffsets[fieldIndex]) = value;
}
template <> void PointCloudAccessor::setField(uint32_t pointIndex, uint32_t fieldIndex, float value) {
    *reinterpret_cast<float*>(pcl->data.data() + pointIndex * pcl->point_step + fieldOffsets[fieldIndex]) = value;
}

} // namespace point_cloud_color
