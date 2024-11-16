#include "pointcloud_processor/pointcloud_processor.hpp"
#include <unordered_map>
#include <cmath>
#include <cstring>

PointCloudProcessor::PointCloudProcessor(const Parameters &params)
    : params_(params)
{
}

std::vector<Point3D> PointCloudProcessor::process_pointcloud(const sensor_msgs::msg::PointCloud2 &cloud_msg)
{
  auto points = PC2_to_vector(cloud_msg);
  auto transformed_points = axis_image2robot(points);
  auto filtered_points = filter_points(transformed_points);
  voxel_downsample(filtered_points);
  return filtered_points;
}

std::vector<Point3D> PointCloudProcessor::get_downsampled_points() const
{
  return downsampled_points_;
}

sensor_msgs::msg::PointCloud2 PointCloudProcessor::vector_to_PC2(const std::vector<Point3D> &points) const
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = "map";
  cloud_msg.height = 1;
  cloud_msg.width = points.size();
  cloud_msg.fields.resize(3);
  cloud_msg.fields[0].name = "x";
  cloud_msg.fields[1].name = "y";
  cloud_msg.fields[2].name = "z";

  for (int i = 0; i < 3; ++i)
  {
    cloud_msg.fields[i].offset = i * sizeof(float);
    cloud_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[i].count = 1;
  }

  cloud_msg.point_step = 3 * sizeof(float);
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = true;

  for (size_t i = 0; i < points.size(); ++i)
  {
    size_t data_index = i * cloud_msg.point_step;
    memcpy(&cloud_msg.data[data_index], &points[i].x, sizeof(float));
    memcpy(&cloud_msg.data[data_index + sizeof(float)], &points[i].y, sizeof(float));
    memcpy(&cloud_msg.data[data_index + 2 * sizeof(float)], &points[i].z, sizeof(float));
  }

  return cloud_msg;
}

std::vector<Point3D> PointCloudProcessor::PC2_to_vector(const sensor_msgs::msg::PointCloud2 &cloud_msg) const
{
  std::vector<Point3D> points;
  size_t num_points = cloud_msg.width * cloud_msg.height;
  points.reserve(num_points);

  size_t point_step = cloud_msg.point_step;
  size_t x_offset = cloud_msg.fields[0].offset;
  size_t y_offset = cloud_msg.fields[1].offset;
  size_t z_offset = cloud_msg.fields[2].offset;

  for (size_t i = 0; i < num_points; ++i)
  {
    size_t data_index = i * point_step;
    Point3D point;
    memcpy(&point.x, &cloud_msg.data[data_index + x_offset], sizeof(float));
    memcpy(&point.y, &cloud_msg.data[data_index + y_offset], sizeof(float));
    memcpy(&point.z, &cloud_msg.data[data_index + z_offset], sizeof(float));
    points.push_back(point);
  }

  return points;
}

std::vector<Point3D> PointCloudProcessor::axis_image2robot(const std::vector<Point3D> &input) const
{
  std::vector<Point3D> swapped;
  swapped.reserve(input.size());

  for (const auto &point : input)
  {
    Point3D swapped_point;
    swapped_point.x = point.z;
    swapped_point.y = point.x;
    swapped_point.z = -point.y;
    swapped.push_back(swapped_point);
  }

  return swapped;
}

std::vector<Point3D> PointCloudProcessor::filter_points(const std::vector<Point3D> &input) const
{
  std::vector<Point3D> output;
  for (const auto &point : input)
  {
    if (point.x != 0.0f && point.y != 0.0f && point.z != 0.0f &&
        point.x >= params_.min_x && point.x <= params_.max_x &&
        point.y >= params_.min_y && point.y <= params_.max_y &&
        point.z >= params_.min_z && point.z <= params_.max_z)
    {
      output.push_back(point);
    }
  }
  return output;
}

void PointCloudProcessor::voxel_downsample(const std::vector<Point3D> &input)
{
  std::unordered_map<std::string, std::vector<Point3D>> voxel_map;
  downsampled_points_.clear();

  for (const auto &point : input)
  {
    int voxel_x = static_cast<int>(std::floor(point.x / params_.D_voxel_size_x));
    int voxel_y = static_cast<int>(std::floor(point.y / params_.D_voxel_size_y));
    int voxel_z = static_cast<int>(std::floor(point.z / params_.D_voxel_size_z));
    std::string key = std::to_string(voxel_x) + "_" + std::to_string(voxel_y) + "_" + std::to_string(voxel_z);
    voxel_map[key].push_back(point);
  }

  for (const auto &voxel : voxel_map)
  {
    const auto &points = voxel.second;
    if (points.size() < 4)
      continue;

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    for (const auto &p : points)
    {
      sum_x += p.x;
      sum_y += p.y;
      sum_z += p.z;
    }

    Point3D centroid;
    centroid.x = sum_x / points.size();
    centroid.y = sum_y / points.size();
    centroid.z = sum_z / points.size();
    downsampled_points_.push_back(centroid);
  }
}