#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <cmath>

int main(int argc, char ** argv) {
  Eigen::Matrix3d lidar2ins_r = Eigen::Matrix3d::Identity();

  lidar2ins_r << 0.999856, 0.0152593, 0.00758859,
               -0.0152559, 0.999882, -0.000505459,
               -0.00759539, 0.000389579, 0.999971;
  
  Eigen::Vector3d lidar2ins_t(0.665661, -0.46954, 2.057);

  Eigen::Matrix3d cam2ins_r = Eigen::Matrix3d::Identity();

  cam2ins_r << -0.003024051765609291, -0.033072407583913194, 0.9994483833432932,
             -0.9997914943361127, 0.020283664727763707, -0.002353890051476679,
             -0.020194627109250664, -0.9992471109799693, -0.03312685065174658;
  
  Eigen::Vector3d cam2ins_t(1.227716, -0.475126, 1.821363);

  Eigen::Matrix3d lidar2cam_r = cam2ins_r.inverse() * lidar2ins_r;

  Eigen::Vector3d lidar2cam_t = cam2ins_r.inverse() * (lidar2ins_t - cam2ins_t);

  std::cout << "----------------------------------------" << std::endl;
  Eigen::Quaterniond cam2lidar_q(lidar2cam_r);
  std::cout << cam2lidar_q.x() << ", " << cam2lidar_q.y() << ", " << cam2lidar_q.z() << ", " << cam2lidar_q.w() << std::endl;
  std::cout << "----------------------------------------" << std::endl;
  std::cout << lidar2cam_t[0] << ", " << lidar2cam_t[1] << ", " << lidar2cam_t[2] << std::endl;

  std::cout << lidar2cam_r(0, 0) << ", " << lidar2cam_r(0, 1) << ", " << lidar2cam_r(0, 2) << ", "
            << lidar2cam_r(1, 0) << ", " << lidar2cam_r(1, 1) << ", " << lidar2cam_r(1, 2) << ", "
            << lidar2cam_r(2, 0) << ", " << lidar2cam_r(2, 1) << ", " << lidar2cam_r(2, 2) << std::endl;
 
  return 0;

}
