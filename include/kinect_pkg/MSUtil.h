// Based on k4a_ros_device.cpp
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// Updates by Justin W. Hart.

#include <k4a/k4a.hpp>
#include <sensor_msgs/PointCloud2.h>

k4a_result_t fillPointCloud(const k4a::image& pointcloud_image, sensor_msgs::PointCloud2 *point_cloud);
k4a_result_t fillColorPointCloud(const k4a::image& pointcloud_image, const k4a::image& color_image, sensor_msgs::PointCloud2 *point_cloud);