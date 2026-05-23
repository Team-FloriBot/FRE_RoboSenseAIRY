#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

class GroundSegmentationNode : public rclcpp::Node
{
public:
  GroundSegmentationNode() : Node("ground_segmentation_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/rslidar_points");
    aligned_topic_ = declare_parameter<std::string>("aligned_topic", "/aligned_points");
    ground_topic_ = declare_parameter<std::string>("ground_topic", "/ground_points");
    nonground_topic_ = declare_parameter<std::string>("nonground_topic", "/nonground_points");
    crop_topic_ = declare_parameter<std::string>("crop_topic", "/crop_points");
    obstacle_topic_ = declare_parameter<std::string>("obstacle_topic", "/obstacle_points");

    parent_frame_ = declare_parameter<std::string>("parent_frame", "base_link");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "rslidar");

    mount_x_ = declare_parameter<double>("mount_x", 0.0);
    mount_y_ = declare_parameter<double>("mount_y", 0.0);
    mount_z_ = declare_parameter<double>("mount_z", 0.50);

    initial_pitch_deg_ = declare_parameter<double>("initial_pitch_deg", -20.0);
    initial_roll_deg_ = declare_parameter<double>("initial_roll_deg", 0.0);

    leveling_pitch_sign_ = declare_parameter<double>("leveling_pitch_sign", 1.0);
    leveling_roll_sign_ = declare_parameter<double>("leveling_roll_sign", 1.0);
    leveling_pitch_gain_ = declare_parameter<double>("leveling_pitch_gain", 1.0);
    leveling_roll_gain_ = declare_parameter<double>("leveling_roll_gain", 1.0);

    use_ring_filter_ = declare_parameter<bool>("use_ring_filter", false);
    ring_min_ = declare_parameter<int>("ring_min", 0);
    ring_max_ = declare_parameter<int>("ring_max", 255);
    allowed_rings_raw_ =
      declare_parameter<std::vector<int64_t>>("allowed_rings", std::vector<int64_t>{});
    buildAllowedRingSet();

    publish_aligned_cloud_ =
      declare_parameter<bool>("publish_aligned_cloud", true);

    use_local_ground_leveling_ =
      declare_parameter<bool>("use_local_ground_leveling", true);
    leveling_update_every_n_frames_ =
      declare_parameter<int>("leveling_update_every_n_frames", 1);
    leveling_stride_ =
      declare_parameter<int>("leveling_stride", 4);
    leveling_alpha_ =
      declare_parameter<double>("leveling_alpha", 0.25);

    iterative_leveling_iterations_ =
      declare_parameter<int>("iterative_leveling_iterations", 4);
    iterative_leveling_step_gain_ =
      declare_parameter<double>("iterative_leveling_step_gain", 1.0);
    iterative_leveling_convergence_deg_ =
      declare_parameter<double>("iterative_leveling_convergence_deg", 0.2);

    max_pitch_correction_deg_ =
      declare_parameter<double>("max_pitch_correction_deg", 35.0);
    max_roll_correction_deg_ =
      declare_parameter<double>("max_roll_correction_deg", 35.0);

    leveling_roi_x_min_ = declare_parameter<double>("leveling_roi_x_min", 0.2);
    leveling_roi_x_max_ = declare_parameter<double>("leveling_roi_x_max", 2.5);
    leveling_roi_y_min_ = declare_parameter<double>("leveling_roi_y_min", -1.2);
    leveling_roi_y_max_ = declare_parameter<double>("leveling_roi_y_max", 1.2);
    leveling_roi_z_min_ = declare_parameter<double>("leveling_roi_z_min", -1.2);
    leveling_roi_z_max_ = declare_parameter<double>("leveling_roi_z_max", 2.0);

    leveling_grid_resolution_ =
      declare_parameter<double>("leveling_grid_resolution", 0.10);
    leveling_min_cells_ =
      declare_parameter<int>("leveling_min_cells", 6);
    leveling_neighbor_radius_ =
      declare_parameter<int>("leveling_neighbor_radius", 1);

    roi_x_min_ = declare_parameter<double>("roi_x_min", 0.0);
    roi_x_max_ = declare_parameter<double>("roi_x_max", 6.0);
    roi_y_min_ = declare_parameter<double>("roi_y_min", -2.0);
    roi_y_max_ = declare_parameter<double>("roi_y_max", 2.0);
    roi_z_min_ = declare_parameter<double>("roi_z_min", -1.0);
    roi_z_max_ = declare_parameter<double>("roi_z_max", 2.0);

    grid_resolution_ = declare_parameter<double>("grid_resolution", 0.08);
    min_points_per_cell_ = declare_parameter<int>("min_points_per_cell", 2);
    local_plane_neighbor_radius_ =
      declare_parameter<int>("local_plane_neighbor_radius", 1);
    local_plane_min_cells_ =
      declare_parameter<int>("local_plane_min_cells", 5);

    base_ground_threshold_ = declare_parameter<double>("base_ground_threshold", 0.05);
    distance_threshold_coeff_ = declare_parameter<double>("distance_threshold_coeff", 0.01);
    negative_outlier_threshold_ = declare_parameter<double>("negative_outlier_threshold", 0.10);
    obstacle_height_span_threshold_ =
      declare_parameter<double>("obstacle_height_span_threshold", 0.12);
    ground_band_height_ =
      declare_parameter<double>("ground_band_height", 0.04);

    enable_crop_obstacle_split_ =
      declare_parameter<bool>("enable_crop_obstacle_split", true);

    cluster_tolerance_ =
      declare_parameter<double>("cluster_tolerance", 0.24);
    cluster_min_size_ =
      declare_parameter<int>("cluster_min_size", 2);
    cluster_max_size_ =
      declare_parameter<int>("cluster_max_size", 20000);

    crop_min_height_ =
      declare_parameter<double>("crop_min_height", 0.04);
    crop_max_height_ =
      declare_parameter<double>("crop_max_height", 1.20);
    crop_max_width_ =
      declare_parameter<double>("crop_max_width", 1.20);
    crop_max_depth_ =
      declare_parameter<double>("crop_max_depth", 1.20);
    crop_max_ground_offset_ =
      declare_parameter<double>("crop_max_ground_offset", 0.45);
    crop_max_base_area_ =
      declare_parameter<double>("crop_max_base_area", 0.45);

    nx_ = static_cast<int>(std::ceil((roi_x_max_ - roi_x_min_) / grid_resolution_));
    ny_ = static_cast<int>(std::ceil((roi_y_max_ - roi_y_min_) / grid_resolution_));

    leveling_nx_ = static_cast<int>(
      std::ceil((leveling_roi_x_max_ - leveling_roi_x_min_) / leveling_grid_resolution_));
    leveling_ny_ = static_cast<int>(
      std::ceil((leveling_roi_y_max_ - leveling_roi_y_min_) / leveling_grid_resolution_));

    estimated_roll_correction_rad_ = 0.0;
    estimated_pitch_correction_rad_ = 0.0;
    frame_counter_ = 0;
    warned_missing_ring_ = false;

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&GroundSegmentationNode::cloudCallback, this, std::placeholders::_1));

    auto out_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(aligned_topic_, out_qos);
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ground_topic_, out_qos);
    nonground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(nonground_topic_, out_qos);
    crop_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(crop_topic_, out_qos);
    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic_, out_qos);

    RCLCPP_INFO(get_logger(), "GroundSegmentationNode gestartet");
    RCLCPP_INFO(get_logger(), "Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Aligned topic: %s", aligned_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Ground topic: %s", ground_topic_.c_str());
    RCLCPP_INFO(get_logger(), "NonGround topic: %s", nonground_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Crop topic: %s", crop_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Obstacle topic: %s", obstacle_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Segmentation frame: %s", parent_frame_.c_str());
    RCLCPP_INFO(
      get_logger(), "Initial pitch=%.2f deg, roll=%.2f deg",
      initial_pitch_deg_, initial_roll_deg_);
  }

private:
  struct RawPoint
  {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    int ring = -1;
  };

  struct Cell
  {
    bool valid = false;
    float min_z = std::numeric_limits<float>::infinity();
    float max_z = -std::numeric_limits<float>::infinity();
    int count = 0;
    float smoothed_z = std::numeric_limits<float>::infinity();
  };

  struct Plane
  {
    bool valid = false;
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
  };

  struct ClusterFeatures
  {
    float min_x = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    float min_z = std::numeric_limits<float>::infinity();
    float max_z = -std::numeric_limits<float>::infinity();
    std::size_t point_count = 0;

    float width() const { return max_x - min_x; }
    float depth() const { return max_y - min_y; }
    float height() const { return max_z - min_z; }
    float base_area() const { return width() * depth(); }
  };

  void buildAllowedRingSet()
  {
    allowed_rings_.clear();
    for (const auto & v : allowed_rings_raw_) {
      allowed_rings_.insert(static_cast<int>(v));
    }
  }

  bool pointPassesRingFilter(int ring) const
  {
    if (!use_ring_filter_) {
      return true;
    }
    if (ring < 0) {
      return false;
    }
    if (!allowed_rings_.empty()) {
      return allowed_rings_.count(ring) > 0;
    }
    return ring >= ring_min_ && ring <= ring_max_;
  }

  bool cloudHasField(
    const sensor_msgs::msg::PointCloud2 & msg,
    const std::string & field_name) const
  {
    for (const auto & f : msg.fields) {
      if (f.name == field_name) {
        return true;
      }
    }
    return false;
  }

  std::vector<RawPoint> extractFilteredRawPoints(const sensor_msgs::msg::PointCloud2 & msg)
  {
    std::vector<RawPoint> points;
    points.reserve(static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height));

    const bool has_ring = cloudHasField(msg, "ring");
    if (!has_ring && !warned_missing_ring_) {
      RCLCPP_WARN(get_logger(), "PointCloud2 hat kein 'ring'-Feld. Ringfilter wird ignoriert.");
      warned_missing_ring_ = true;
    }

    if (has_ring) {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
      sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(msg, "ring");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_ring) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
          continue;
        }

        const int ring = static_cast<int>(*iter_ring);
        if (!pointPassesRingFilter(ring)) {
          continue;
        }

        RawPoint p;
        p.x = *iter_x;
        p.y = *iter_y;
        p.z = *iter_z;
        p.ring = ring;
        points.push_back(p);
      }
    } else {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
          continue;
        }

        RawPoint p;
        p.x = *iter_x;
        p.y = *iter_y;
        p.z = *iter_z;
        p.ring = -1;
        points.push_back(p);
      }
    }

    return points;
  }

  inline bool inROI(const pcl::PointXYZ & p) const
  {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           p.x >= roi_x_min_ && p.x <= roi_x_max_ &&
           p.y >= roi_y_min_ && p.y <= roi_y_max_ &&
           p.z >= roi_z_min_ && p.z <= roi_z_max_;
  }

  inline bool inLevelingROI(const pcl::PointXYZ & p) const
  {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           p.x >= leveling_roi_x_min_ && p.x <= leveling_roi_x_max_ &&
           p.y >= leveling_roi_y_min_ && p.y <= leveling_roi_y_max_ &&
           p.z >= leveling_roi_z_min_ && p.z <= leveling_roi_z_max_;
  }

  inline int ix(float x) const
  {
    return static_cast<int>((x - roi_x_min_) / grid_resolution_);
  }

  inline int iy(float y) const
  {
    return static_cast<int>((y - roi_y_min_) / grid_resolution_);
  }

  inline int cellIndex(int cx, int cy) const
  {
    return cy * nx_ + cx;
  }

  inline int levelingIx(float x) const
  {
    return static_cast<int>((x - leveling_roi_x_min_) / leveling_grid_resolution_);
  }

  inline int levelingIy(float y) const
  {
    return static_cast<int>((y - leveling_roi_y_min_) / leveling_grid_resolution_);
  }

  inline int levelingCellIndex(int cx, int cy) const
  {
    return cy * leveling_nx_ + cx;
  }

  static float median(std::vector<float> & values)
  {
    if (values.empty()) {
      return std::numeric_limits<float>::infinity();
    }
    const auto mid = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), mid, values.end());
    return *mid;
  }

  pcl::PointXYZ invertZOnly(const RawPoint & p_sensor) const
  {
    pcl::PointXYZ out;
    out.x = p_sensor.x;
    out.y = -p_sensor.y;
    out.z = -p_sensor.z;
    return out;
  }

  pcl::PointXYZ transformWithAngles(
    const pcl::PointXYZ & p_up,
    double roll_rad,
    double pitch_rad) const
  {
    tf2::Quaternion q;
    q.setRPY(roll_rad, pitch_rad, 0.0);
    q.normalize();

    tf2::Transform t;
    t.setOrigin(tf2::Vector3(mount_x_, mount_y_, mount_z_));
    t.setRotation(q);

    tf2::Vector3 p_in(p_up.x, p_up.y, p_up.z);
    tf2::Vector3 p_out = t * p_in;

    pcl::PointXYZ out;
    out.x = static_cast<float>(p_out.x());
    out.y = static_cast<float>(p_out.y());
    out.z = static_cast<float>(p_out.z());
    return out;
  }

  bool solve3x3(std::array<std::array<double, 4>, 3> & m, double & x, double & y, double & z) const
  {
    for (int col = 0; col < 3; ++col) {
      int pivot = col;
      for (int r = col + 1; r < 3; ++r) {
        if (std::abs(m[r][col]) > std::abs(m[pivot][col])) {
          pivot = r;
        }
      }

      if (std::abs(m[pivot][col]) < 1e-9) {
        return false;
      }

      if (pivot != col) {
        std::swap(m[pivot], m[col]);
      }

      const double div = m[col][col];
      for (int c = col; c < 4; ++c) {
        m[col][c] /= div;
      }

      for (int r = 0; r < 3; ++r) {
        if (r == col) {
          continue;
        }
        const double factor = m[r][col];
        for (int c = col; c < 4; ++c) {
          m[r][c] -= factor * m[col][c];
        }
      }
    }

    x = m[0][3];
    y = m[1][3];
    z = m[2][3];
    return true;
  }

  bool fitPlaneZ(const std::vector<std::array<double, 3>> & pts, Plane & plane) const
  {
    if (pts.size() < 3) {
      plane.valid = false;
      return false;
    }

    double sx = 0.0, sy = 0.0, sz = 0.0;
    double sxx = 0.0, syy = 0.0, sxy = 0.0;
    double sxz = 0.0, syz = 0.0;
    const double n = static_cast<double>(pts.size());

    for (const auto & p : pts) {
      const double x = p[0];
      const double y = p[1];
      const double z = p[2];
      sx += x;
      sy += y;
      sz += z;
      sxx += x * x;
      syy += y * y;
      sxy += x * y;
      sxz += x * z;
      syz += y * z;
    }

    std::array<std::array<double, 4>, 3> m{{
      {{sxx, sxy, sx,  sxz}},
      {{sxy, syy, sy,  syz}},
      {{sx,  sy,  n,   sz }}
    }};

    double a = 0.0, b = 0.0, c = 0.0;
    if (!solve3x3(m, a, b, c)) {
      plane.valid = false;
      return false;
    }

    plane.valid = true;
    plane.a = a;
    plane.b = b;
    plane.c = c;
    return true;
  }

  bool collectLevelingPlanePoints(
    const std::vector<RawPoint> & raw_points,
    double roll_rad,
    double pitch_rad,
    std::vector<std::array<double, 3>> & plane_pts) const
  {
    std::vector<Cell> level_cells(leveling_nx_ * leveling_ny_);
    const std::size_t stride = static_cast<std::size_t>(std::max(1, leveling_stride_));

    for (std::size_t i = 0; i < raw_points.size(); i += stride) {
      const pcl::PointXYZ p_up = invertZOnly(raw_points[i]);
      const pcl::PointXYZ p_guess = transformWithAngles(p_up, roll_rad, pitch_rad);

      if (!inLevelingROI(p_guess)) {
        continue;
      }

      const int cx = levelingIx(p_guess.x);
      const int cy = levelingIy(p_guess.y);
      if (cx < 0 || cx >= leveling_nx_ || cy < 0 || cy >= leveling_ny_) {
        continue;
      }

      Cell & c = level_cells[levelingCellIndex(cx, cy)];
      c.count++;
      if (p_guess.z < c.min_z) {
        c.min_z = p_guess.z;
      }
      if (p_guess.z > c.max_z) {
        c.max_z = p_guess.z;
      }
    }

    plane_pts.clear();
    plane_pts.reserve(level_cells.size());

    for (int cy = 0; cy < leveling_ny_; ++cy) {
      for (int cx = 0; cx < leveling_nx_; ++cx) {
        const Cell & c = level_cells[levelingCellIndex(cx, cy)];
        if (c.count < 1 || !std::isfinite(c.min_z)) {
          continue;
        }

        std::vector<float> neigh;
        for (int dy = -leveling_neighbor_radius_; dy <= leveling_neighbor_radius_; ++dy) {
          for (int dx = -leveling_neighbor_radius_; dx <= leveling_neighbor_radius_; ++dx) {
            const int nx = cx + dx;
            const int ny = cy + dy;
            if (nx < 0 || nx >= leveling_nx_ || ny < 0 || ny >= leveling_ny_) {
              continue;
            }
            const Cell & n = level_cells[levelingCellIndex(nx, ny)];
            if (n.count > 0 && std::isfinite(n.min_z)) {
              neigh.push_back(n.min_z);
            }
          }
        }

        float z_use = c.min_z;
        if (!neigh.empty()) {
          z_use = median(neigh);
        }

        const double x_center =
          leveling_roi_x_min_ + (static_cast<double>(cx) + 0.5) * leveling_grid_resolution_;
        const double y_center =
          leveling_roi_y_min_ + (static_cast<double>(cy) + 0.5) * leveling_grid_resolution_;

        plane_pts.push_back({x_center, y_center, static_cast<double>(z_use)});
      }
    }

    return static_cast<int>(plane_pts.size()) >= leveling_min_cells_;
  }

  void estimateLocalGroundLeveling(const std::vector<RawPoint> & raw_points)
  {
    if (!use_local_ground_leveling_) {
      estimated_roll_correction_rad_ = 0.0;
      estimated_pitch_correction_rad_ = 0.0;
      return;
    }

    frame_counter_++;
    const std::size_t update_n =
      static_cast<std::size_t>(std::max(1, leveling_update_every_n_frames_));
    if ((frame_counter_ % update_n) != 0) {
      return;
    }

    double work_roll_corr = estimated_roll_correction_rad_;
    double work_pitch_corr = estimated_pitch_correction_rad_;

    const double max_pitch_corr = max_pitch_correction_deg_ * M_PI / 180.0;
    const double max_roll_corr = max_roll_correction_deg_ * M_PI / 180.0;
    const double convergence_rad = iterative_leveling_convergence_deg_ * M_PI / 180.0;

    std::vector<std::array<double, 3>> plane_pts;
    Plane plane;

    bool any_success = false;
    double last_measured_pitch = 0.0;
    double last_measured_roll = 0.0;
    std::size_t last_cell_count = 0;

    for (int iter = 0; iter < std::max(1, iterative_leveling_iterations_); ++iter) {
      const double roll_rad = initial_roll_deg_ * M_PI / 180.0 + work_roll_corr;
      const double pitch_rad = initial_pitch_deg_ * M_PI / 180.0 + work_pitch_corr;

      if (!collectLevelingPlanePoints(raw_points, roll_rad, pitch_rad, plane_pts)) {
        break;
      }
      last_cell_count = plane_pts.size();

      if (!fitPlaneZ(plane_pts, plane) || !plane.valid) {
        break;
      }

      const double measured_pitch =
        leveling_pitch_sign_ * leveling_pitch_gain_ * std::atan(plane.a);
      const double measured_roll =
        leveling_roll_sign_ * leveling_roll_gain_ * (-std::atan(plane.b));

      last_measured_pitch = measured_pitch;
      last_measured_roll = measured_roll;
      any_success = true;

      const double delta_pitch = iterative_leveling_step_gain_ * measured_pitch;
      const double delta_roll = iterative_leveling_step_gain_ * measured_roll;

      work_pitch_corr += delta_pitch;
      work_roll_corr += delta_roll;

      work_pitch_corr = std::max(-max_pitch_corr, std::min(work_pitch_corr, max_pitch_corr));
      work_roll_corr = std::max(-max_roll_corr, std::min(work_roll_corr, max_roll_corr));

      if (std::abs(delta_pitch) < convergence_rad && std::abs(delta_roll) < convergence_rad) {
        break;
      }
    }

    if (!any_success) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Iteratives leveling: keine gültige lokale Bodenebene");
      return;
    }

    estimated_pitch_correction_rad_ =
      (1.0 - leveling_alpha_) * estimated_pitch_correction_rad_ +
      leveling_alpha_ * work_pitch_corr;

    estimated_roll_correction_rad_ =
      (1.0 - leveling_alpha_) * estimated_roll_correction_rad_ +
      leveling_alpha_ * work_roll_corr;

    estimated_pitch_correction_rad_ =
      std::max(-max_pitch_corr, std::min(estimated_pitch_correction_rad_, max_pitch_corr));
    estimated_roll_correction_rad_ =
      std::max(-max_roll_corr, std::min(estimated_roll_correction_rad_, max_roll_corr));

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Iterative leveling: cells=%zu last_pitch=%.2f last_roll=%.2f corr_pitch=%.2f corr_roll=%.2f final_pitch=%.2f final_roll=%.2f",
      last_cell_count,
      last_measured_pitch * 180.0 / M_PI,
      last_measured_roll * 180.0 / M_PI,
      estimated_pitch_correction_rad_ * 180.0 / M_PI,
      estimated_roll_correction_rad_ * 180.0 / M_PI,
      initial_pitch_deg_ + estimated_pitch_correction_rad_ * 180.0 / M_PI,
      initial_roll_deg_ + estimated_roll_correction_rad_ * 180.0 / M_PI);
  }

  ClusterFeatures computeClusterFeatures(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const pcl::PointIndices & indices) const
  {
    ClusterFeatures f;
    f.point_count = indices.indices.size();

    for (const int idx : indices.indices) {
      const auto & p = cloud->points[idx];
      f.min_x = std::min(f.min_x, p.x);
      f.max_x = std::max(f.max_x, p.x);
      f.min_y = std::min(f.min_y, p.y);
      f.max_y = std::max(f.max_y, p.y);
      f.min_z = std::min(f.min_z, p.z);
      f.max_z = std::max(f.max_z, p.z);
    }

    return f;
  }

  bool isCropCluster(const ClusterFeatures & f) const
  {
    const double h = static_cast<double>(f.height());
    const bool height_ok = h >= crop_min_height_ && h <= crop_max_height_;
    const bool attached_to_ground =
      static_cast<double>(f.min_z) <= crop_max_ground_offset_;

    return height_ok && attached_to_ground;
  }

  void publishCloud(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & frame_id)
  {
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(cloud, out_msg);
    out_msg.header.stamp = stamp;
    out_msg.header.frame_id = frame_id;
    pub->publish(out_msg);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto raw_points = extractFilteredRawPoints(*msg);
    estimateLocalGroundLeveling(raw_points);

    const double final_roll_rad =
      initial_roll_deg_ * M_PI / 180.0 + estimated_roll_correction_rad_;
    const double final_pitch_rad =
      initial_pitch_deg_ * M_PI / 180.0 + estimated_pitch_correction_rad_;

    std::vector<Cell> cells(nx_ * ny_);
    std::vector<pcl::PointXYZ> roi_points_parent;
    roi_points_parent.reserve(raw_points.size());

    pcl::PointCloud<pcl::PointXYZ> aligned_cloud_parent;
    if (publish_aligned_cloud_) {
      aligned_cloud_parent.reserve(raw_points.size());
    }

    for (const auto & p_sensor : raw_points) {
      const pcl::PointXYZ p_up = invertZOnly(p_sensor);
      const pcl::PointXYZ p_parent =
        transformWithAngles(p_up, final_roll_rad, final_pitch_rad);

      if (publish_aligned_cloud_) {
        aligned_cloud_parent.push_back(p_parent);
      }

      if (!inROI(p_parent)) {
        continue;
      }

      const int cx = ix(p_parent.x);
      const int cy = iy(p_parent.y);
      if (cx < 0 || cx >= nx_ || cy < 0 || cy >= ny_) {
        continue;
      }

      Cell & cell = cells[cellIndex(cx, cy)];
      cell.count++;
      if (p_parent.z < cell.min_z) {
        cell.min_z = p_parent.z;
      }
      if (p_parent.z > cell.max_z) {
        cell.max_z = p_parent.z;
      }

      roi_points_parent.push_back(p_parent);
    }

    if (publish_aligned_cloud_) {
      publishCloud(aligned_cloud_parent, aligned_pub_, msg->header.stamp, parent_frame_);
    }

    for (auto & c : cells) {
      if (c.count >= min_points_per_cell_ && std::isfinite(c.min_z)) {
        c.valid = true;
      }
    }

    for (int cy = 0; cy < ny_; ++cy) {
      for (int cx = 0; cx < nx_; ++cx) {
        Cell & c = cells[cellIndex(cx, cy)];
        std::vector<float> neighbors;
        neighbors.reserve(25);

        for (int dy = -local_plane_neighbor_radius_; dy <= local_plane_neighbor_radius_; ++dy) {
          for (int dx = -local_plane_neighbor_radius_; dx <= local_plane_neighbor_radius_; ++dx) {
            const int nx = cx + dx;
            const int ny = cy + dy;
            if (nx < 0 || nx >= this->nx_ || ny < 0 || ny >= this->ny_) {
              continue;
            }
            const Cell & n = cells[cellIndex(nx, ny)];
            if (n.valid && std::isfinite(n.min_z)) {
              neighbors.push_back(n.min_z);
            }
          }
        }

        if (!neighbors.empty()) {
          c.smoothed_z = median(neighbors);
          c.valid = true;
        }
      }
    }

    std::vector<Plane> local_planes(static_cast<std::size_t>(nx_ * ny_));

    for (int cy = 0; cy < ny_; ++cy) {
      for (int cx = 0; cx < nx_; ++cx) {
        std::vector<std::array<double, 3>> plane_pts;
        plane_pts.reserve(25);

        for (int dy = -local_plane_neighbor_radius_; dy <= local_plane_neighbor_radius_; ++dy) {
          for (int dx = -local_plane_neighbor_radius_; dx <= local_plane_neighbor_radius_; ++dx) {
            const int nx = cx + dx;
            const int ny = cy + dy;
            if (nx < 0 || nx >= this->nx_ || ny < 0 || ny >= this->ny_) {
              continue;
            }

            const Cell & n = cells[cellIndex(nx, ny)];
            if (!n.valid || !std::isfinite(n.smoothed_z)) {
              continue;
            }

            const double x_center =
              roi_x_min_ + (static_cast<double>(nx) + 0.5) * grid_resolution_;
            const double y_center =
              roi_y_min_ + (static_cast<double>(ny) + 0.5) * grid_resolution_;

            plane_pts.push_back({x_center, y_center, static_cast<double>(n.smoothed_z)});
          }
        }

        Plane plane;
        if (static_cast<int>(plane_pts.size()) >= local_plane_min_cells_) {
          fitPlaneZ(plane_pts, plane);
        }
        local_planes[static_cast<std::size_t>(cellIndex(cx, cy))] = plane;
      }
    }

    pcl::PointCloud<pcl::PointXYZ> ground_cloud_parent;
    pcl::PointCloud<pcl::PointXYZ> nonground_cloud_parent;
    ground_cloud_parent.reserve(roi_points_parent.size());
    nonground_cloud_parent.reserve(roi_points_parent.size());

    for (const auto & p : roi_points_parent) {
      const int cx = ix(p.x);
      const int cy = iy(p.y);

      if (cx < 0 || cx >= nx_ || cy < 0 || cy >= ny_) {
        nonground_cloud_parent.push_back(p);
        continue;
      }

      const Plane & plane = local_planes[static_cast<std::size_t>(cellIndex(cx, cy))];
      const Cell & c = cells[cellIndex(cx, cy)];

      if (!plane.valid || !c.valid) {
        nonground_cloud_parent.push_back(p);
        continue;
      }

      const double z_plane =
        plane.a * static_cast<double>(p.x) +
        plane.b * static_cast<double>(p.y) +
        plane.c;

      const double dz = static_cast<double>(p.z) - z_plane;
      const double range_xy = std::hypot(p.x, p.y);
      const double adaptive_thresh =
        base_ground_threshold_ + distance_threshold_coeff_ * range_xy;
      const double cell_span = static_cast<double>(c.max_z) - static_cast<double>(c.min_z);

      if (cell_span > obstacle_height_span_threshold_) {
        if (dz >= -negative_outlier_threshold_ && dz <= ground_band_height_) {
          ground_cloud_parent.push_back(p);
        } else {
          nonground_cloud_parent.push_back(p);
        }
        continue;
      }

      if (dz >= -negative_outlier_threshold_ && dz <= adaptive_thresh) {
        ground_cloud_parent.push_back(p);
      } else {
        nonground_cloud_parent.push_back(p);
      }
    }

    publishCloud(ground_cloud_parent, ground_pub_, msg->header.stamp, parent_frame_);
    publishCloud(nonground_cloud_parent, nonground_pub_, msg->header.stamp, parent_frame_);

    pcl::PointCloud<pcl::PointXYZ> crop_cloud_parent;
    pcl::PointCloud<pcl::PointXYZ> obstacle_cloud_parent;

    if (enable_crop_obstacle_split_ && !nonground_cloud_parent.empty()) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr nonground_ptr(
        new pcl::PointCloud<pcl::PointXYZ>(nonground_cloud_parent));

      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(nonground_ptr);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(cluster_tolerance_);
      ec.setMinClusterSize(cluster_min_size_);
      ec.setMaxClusterSize(cluster_max_size_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(nonground_ptr);
      ec.extract(cluster_indices);

      for (const auto & cluster : cluster_indices) {
        const ClusterFeatures f = computeClusterFeatures(nonground_ptr, cluster);
        const bool crop = isCropCluster(f);

        for (const int idx : cluster.indices) {
          const auto & p = nonground_ptr->points[idx];
          if (crop) {
            crop_cloud_parent.push_back(p);
          } else {
            obstacle_cloud_parent.push_back(p);
          }
        }
      }
    } else {
      obstacle_cloud_parent = nonground_cloud_parent;
    }

    publishCloud(crop_cloud_parent, crop_pub_, msg->header.stamp, parent_frame_);
    publishCloud(obstacle_cloud_parent, obstacle_pub_, msg->header.stamp, parent_frame_);
  }

  std::string input_topic_;
  std::string aligned_topic_;
  std::string ground_topic_;
  std::string nonground_topic_;
  std::string crop_topic_;
  std::string obstacle_topic_;

  std::string parent_frame_;
  std::string sensor_frame_;

  double mount_x_;
  double mount_y_;
  double mount_z_;
  double initial_pitch_deg_;
  double initial_roll_deg_;

  double leveling_pitch_sign_;
  double leveling_roll_sign_;
  double leveling_pitch_gain_;
  double leveling_roll_gain_;

  bool use_ring_filter_;
  int ring_min_;
  int ring_max_;
  std::vector<int64_t> allowed_rings_raw_;
  std::unordered_set<int> allowed_rings_;
  bool warned_missing_ring_;

  bool publish_aligned_cloud_;

  bool use_local_ground_leveling_;
  int leveling_update_every_n_frames_;
  int leveling_stride_;
  double leveling_alpha_;
  int iterative_leveling_iterations_;
  double iterative_leveling_step_gain_;
  double iterative_leveling_convergence_deg_;
  double max_pitch_correction_deg_;
  double max_roll_correction_deg_;
  double estimated_roll_correction_rad_;
  double estimated_pitch_correction_rad_;
  std::size_t frame_counter_;

  double leveling_roi_x_min_;
  double leveling_roi_x_max_;
  double leveling_roi_y_min_;
  double leveling_roi_y_max_;
  double leveling_roi_z_min_;
  double leveling_roi_z_max_;
  double leveling_grid_resolution_;
  int leveling_min_cells_;
  int leveling_neighbor_radius_;

  double roi_x_min_;
  double roi_x_max_;
  double roi_y_min_;
  double roi_y_max_;
  double roi_z_min_;
  double roi_z_max_;

  double grid_resolution_;
  int min_points_per_cell_;
  int local_plane_neighbor_radius_;
  int local_plane_min_cells_;
  double base_ground_threshold_;
  double distance_threshold_coeff_;
  double negative_outlier_threshold_;
  double obstacle_height_span_threshold_;
  double ground_band_height_;

  bool enable_crop_obstacle_split_;
  double cluster_tolerance_;
  int cluster_min_size_;
  int cluster_max_size_;

  double crop_min_height_;
  double crop_max_height_;
  double crop_max_width_;
  double crop_max_depth_;
  double crop_max_ground_offset_;
  double crop_max_base_area_;

  int nx_;
  int ny_;
  int leveling_nx_;
  int leveling_ny_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr crop_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}
