// Based on k4a_ros_device.cpp
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// Updates by Justin W. Hart.

#include <kinect_pkg/MSUtil.h>
#include <sensor_msgs/point_cloud2_iterator.h>


k4a_result_t fillPointCloud(const k4a::image& pointcloud_image, sensor_msgs::PointCloud2Ptr& point_cloud) {
  point_cloud->height = pointcloud_image.get_height_pixels();
  point_cloud->width = pointcloud_image.get_width_pixels();
  point_cloud->is_dense = false;
  point_cloud->is_bigendian = false;

  const size_t point_count = pointcloud_image.get_height_pixels() * pointcloud_image.get_width_pixels();

  sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");

  pcd_modifier.resize(point_count);

  const int16_t* point_cloud_buffer = reinterpret_cast<const int16_t*>(pointcloud_image.get_buffer());

  for (size_t i = 0; i < point_count; i++, ++iter_x, ++iter_y, ++iter_z)
  {
    float z = static_cast<float>(point_cloud_buffer[3 * i + 2]);

    if (z <= 0.0f)
    {
      *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
    }
    else
    {
      constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
      *iter_x = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 0]);
      *iter_y = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 1]);
      *iter_z = kMillimeterToMeter * z;
    }
  }

  return K4A_RESULT_SUCCEEDED;
}
