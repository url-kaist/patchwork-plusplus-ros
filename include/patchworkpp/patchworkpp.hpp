/**
 * @file patchworkpp.hpp
 * @author Seungjae Lee
 * @brief
 * @version 0.1
 * @date 2022-07-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef PATCHWORKPP_H
#define PATCHWORKPP_H

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <Eigen/Dense>
#include <boost/format.hpp>
#include <numeric>

#include <mutex>

#include <patchworkpp/utils.hpp>

#define MARKER_Z_VALUE -2.2
// 5 Color Heat Map
#define UPRIGHT_ENOUGH 0.55     // Green
#define FLAT_ENOUGH 0.2         // Blue Green
#define TOO_HIGH_ELEVATION 0.0  // Blue
#define TOO_TILTED 1.0          // Red

#define NUM_HEURISTIC_MAX_PTS_IN_PATCH 3000

using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

using namespace std;

namespace patchworkpp
{
template <typename PointT>
bool point_z_cmp(PointT a, PointT b)
{
  return a.z < b.z;
}

template <typename PointT>
struct RevertCandidate
{
  int concentric_idx;
  int sector_idx;
  double ground_flatness;
  double line_variable;
  Eigen::Vector4f pc_mean;
  pcl::PointCloud<PointT> regionwise_ground;

  RevertCandidate(int _c_idx, int _s_idx, double _flatness, double _line_var, Eigen::Vector4f _pc_mean, pcl::PointCloud<PointT> _ground)
      : concentric_idx(_c_idx),
        sector_idx(_s_idx),
        ground_flatness(_flatness),
        line_variable(_line_var),
        pc_mean(_pc_mean),
        regionwise_ground(_ground)
  {
  }
};

template <typename PointT>
class PatchWorkpp
{
 public:
  typedef std::vector<pcl::PointCloud<PointT>> Ring;
  typedef std::vector<Ring> Zone;

  PatchWorkpp(ros::NodeHandle *nh);
  ~PatchWorkpp();
  void estimate_ground(pcl::PointCloud<PointT> cloud_in, pcl::PointCloud<PointT> &cloud_ground, pcl::PointCloud<PointT> &cloud_nonground,
                       double &time_taken);

 private:
  // Every private member variable is written with the undescore("_") in its end.

  ros::NodeHandle node_handle_;

  std::recursive_mutex mutex_;

  int num_iter_;
  int num_lpr_;
  int num_min_pts_;
  int num_zones_;
  int num_rings_of_interest_;

  double sensor_height_;
  double th_seeds_;
  double th_dist_;
  double th_seeds_v_;
  double th_dist_v_;
  double max_range_;
  double min_range_;
  double uprightness_thr_;
  double adaptive_seed_selection_margin_;
  double min_range_z2_;  // 12.3625
  double min_range_z3_;  // 22.025
  double min_range_z4_;  // 41.35
  double RNR_ver_angle_thr_;
  double RNR_intensity_thr_;

  bool verbose_;
  bool enable_RNR_;
  bool enable_RVPF_;
  bool enable_TGR_;

  int max_flatness_storage_, max_elevation_storage_;
  std::vector<double> update_flatness_[4];
  std::vector<double> update_elevation_[4];

  float d_;

  VectorXf normal_;
  MatrixXf pnormal_;
  VectorXf singular_values_;
  Eigen::Matrix3f cov_;
  Eigen::Vector4f pc_mean_;

  // For visualization
  bool visualize_;

  vector<int> num_sectors_each_zone_;
  vector<int> num_rings_each_zone_;

  vector<double> sector_sizes_;
  vector<double> ring_sizes_;
  vector<double> min_ranges_;
  vector<double> elevation_thr_;
  vector<double> flatness_thr_;

  vector<Zone> ConcentricZoneModel_;

  jsk_recognition_msgs::PolygonArray poly_list_;

  ros::Publisher PlaneViz, pub_revert_pc, pub_reject_pc, pub_normal, pub_noise, pub_vertical;
  pcl::PointCloud<PointT> revert_pc_, reject_pc_, noise_pc_, vertical_pc_;
  pcl::PointCloud<PointT> ground_pc_;

  pcl::PointCloud<pcl::PointXYZINormal> normals_;

  pcl::PointCloud<PointT> regionwise_ground_, regionwise_nonground_;

  void initialize_zone(Zone &z, int num_sectors, int num_rings);

  void flush_patches_in_zone(Zone &patches, int num_sectors, int num_rings);
  void flush_patches(std::vector<Zone> &czm);

  void pc2czm(const pcl::PointCloud<PointT> &src, std::vector<Zone> &czm);

  void reflected_noise_removal(pcl::PointCloud<PointT> &cloud, pcl::PointCloud<PointT> &cloud_nonground);

  void temporal_ground_revert(pcl::PointCloud<PointT> &cloud_ground, pcl::PointCloud<PointT> &cloud_nonground, std::vector<double> ring_flatness,
                              std::vector<RevertCandidate<PointT>> candidates, int concentric_idx);

  void calc_mean_stdev(std::vector<double> vec, double &mean, double &stdev);

  void update_elevation_thr();
  void update_flatness_thr();

  double xy2theta(const double &x, const double &y);

  double xy2radius(const double &x, const double &y);

  void estimate_plane(const pcl::PointCloud<PointT> &ground);

  void extract_piecewiseground(const int zone_idx, const pcl::PointCloud<PointT> &src, pcl::PointCloud<PointT> &dst,
                               pcl::PointCloud<PointT> &non_ground_dst);

  void extract_initial_seeds(const int zone_idx, const pcl::PointCloud<PointT> &p_sorted, pcl::PointCloud<PointT> &init_seeds);

  void extract_initial_seeds(const int zone_idx, const pcl::PointCloud<PointT> &p_sorted, pcl::PointCloud<PointT> &init_seeds, double th_seed);

  /***
   * For visulization of Ground Likelihood Estimation
   */
  geometry_msgs::PolygonStamped set_polygons(int zone_idx, int r_idx, int theta_idx, int num_split);

  void set_ground_likelihood_estimation_status(const int zone_idx, const int ring_idx, const int concentric_idx, const double z_vec,
                                               const double z_elevation, const double ground_flatness);
};

}  // namespace patchworkpp
#endif