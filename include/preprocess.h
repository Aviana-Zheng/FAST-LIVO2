/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef PREPROCESS_H_
#define PREPROCESS_H_

#include "common_lib.h"
#include <livox_ros_driver/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

enum LiDARFeature
{
  Nor,            // 正常点
  Poss_Plane,     // 可能的平面点
  Real_Plane,     // 确定的平面点
  Edge_Jump,      // 有跨越的边
  Edge_Plane,     // 边上的平面点
  Wire,           // 线段，也许充当了无效点？也就是空间中的小线段
  ZeroPoint       // 无效点，程序中未使用
};
enum Surround
{
  Prev,
  Next
};
enum E_jump
{
  Nr_nor,         // 正常
  Nr_zero,        // 0
  Nr_180,         // 180
  Nr_inf,         // 无穷大 跳变较远
  Nr_blind        // 在盲区？
};

struct orgtype
{
  double range;         // 点云在xy平面离雷达中心的距离
  double dista;         // 当前点与后一个点的距离
  // 假设雷达原点为o 前一个点为M 当前点是A 后一个点是N
  double angle[2];      // 这个是角OAM和角OAN的cos值
  double intersect;     // 这个是角MAN的cos值
  E_jump edj[2];        // 前后两点的类型
  LiDARFeature ftype;   // 点类型
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

/*** Velodyne ***/
namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t t;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t, t, t)(std::uint16_t, ring, ring))
/****************/

/*** Ouster ***/
namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t t;
  std::uint16_t reflectivity;
  uint8_t ring;
  std::uint16_t ambient;
  std::uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                  (std::uint32_t, t, t)(std::uint16_t, reflectivity,
                                                        reflectivity)(std::uint8_t, ring, ring)(std::uint16_t, ambient, ambient)(std::uint32_t, range, range))
/****************/

/*** Hesai_XT32 ***/
namespace xt32_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace xt32_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(xt32_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
/*****************/

/*** Hesai_Pandar128 ***/
namespace Pandar128_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float timestamp;
  uint8_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace Pandar128_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(Pandar128_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, timestamp, timestamp))
/*****************/

class Preprocess
{
public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();

  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128]; // maximum 128 line lidar
  vector<orgtype> typess[128]; // maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS;
  
  double blind, blind_sqr;              // 盲区
  bool feature_enabled, given_offset_time;
  ros::Publisher pub_full, pub_surf, pub_corn;

private:
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void xt32_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void Pandar128_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void l515_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  int plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
typedef std::shared_ptr<Preprocess> PreprocessPtr;

#endif // PREPROCESS_H_