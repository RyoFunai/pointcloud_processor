#pragma once

#include "types.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <memory>

class PointCloudProcessor
{
public:
  PointCloudProcessor(const Parameters &params);

  // 点群全体を処理する関数
  std::vector<Point3D> process_pointcloud(const sensor_msgs::msg::PointCloud2 &cloud_msg);

  // ダウンサンプリングされた点群を取得
  std::vector<Point3D> get_downsampled_points() const;

  // ベクトルからPointCloud2への変換
  sensor_msgs::msg::PointCloud2 vector_to_PC2(const std::vector<Point3D> &points) const;

private:
  struct Voxel
  {
    int x, y, z;

    bool operator==(const Voxel &other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelHash
  {
    std::size_t operator()(const Voxel &voxel) const
    {
      return std::hash<int>()(voxel.x) ^ (std::hash<int>()(voxel.y) << 1) ^ (std::hash<int>()(voxel.z) << 2);
    }
  };

  Parameters params_;
  std::vector<Point3D> downsampled_points_;

  // 内部で使用する関数
  std::vector<Point3D> PC2_to_vector(const sensor_msgs::msg::PointCloud2 &cloud_msg) const;
  std::vector<Point3D> axis_image2robot(const std::vector<Point3D> &input) const;
  std::vector<Point3D> filter_points(const std::vector<Point3D> &input) const;
  void voxel_downsample(const std::vector<Point3D> &input);
};