// Reads the ROS-free EuRoC-style extraction of the ROVTIO dataset and replays it
// as one timestamp-ordered event stream. No ROS, no rosbag.
#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace rovtio_rosfree {

struct Event {
  enum Kind { kImu = 0, kImg = 1 };
  double t = 0.0;  // seconds, already offset-corrected
  Kind kind = kImu;
  int camID = -1;
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();  // kImu, [m s^-2]
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();  // kImu, [rad s^-1]
  std::string path;                               // kImg
};

struct CamSpec {
  std::string dir;      // <seq>/<topic_dir>
  double offset = 0.0;  // added to the image stamp, matching RovioNode's camX_offset
  bool thermal = false; // 16-bit raw counts; must NOT be normalised
};

// Decoded image handed to the filter. Always CV_32FC1.
//   visual : BGR -> gray -> float, 0..255
//   thermal: raw 16-bit counts -> float, ~2800..16383  (ImgUpdate keys its own
//            histogram equalisation off max>255, so pre-scaling would disable it)
struct Frame {
  int camID = -1;
  double t = 0.0;
  cv::Mat img;  // CV_32FC1
};

class DatasetPlayer {
 public:
  DatasetPlayer(const std::string& seq_dir, const std::string& imu_csv,
                const std::vector<CamSpec>& cams, int decode_threads = 3,
                size_t queue_cap = 64);
  ~DatasetPlayer();

  // Parse the CSVs and build the merged event list. Throws on malformed input.
  void load();

  // Start the decode workers. Must follow load().
  void start();

  // Pop the next event in time order. Image events come back decoded.
  // Returns false once the stream is exhausted.
  bool next(Event& ev, Frame& frame);

  size_t size() const { return events_.size(); }
  double t0() const { return events_.empty() ? 0.0 : events_.front().t; }
  double t1() const { return events_.empty() ? 0.0 : events_.back().t; }
  size_t nImu() const { return n_imu_; }
  size_t nImg(int camID) const { return camID < (int)n_img_.size() ? n_img_[camID] : 0; }
  double firstImuTime() const { return first_imu_t_; }
  double firstImgTime(int camID) const {
    return camID < (int)first_img_t_.size() ? first_img_t_[camID] : 0.0;
  }

  static cv::Mat decodeVisual(const std::string& path);
  static cv::Mat decodeThermal(const std::string& path);

 private:
  void worker();

  std::string seq_dir_, imu_csv_;
  std::vector<CamSpec> cams_;
  std::vector<Event> events_;
  size_t n_imu_ = 0;
  std::vector<size_t> n_img_;
  double first_imu_t_ = 0.0;
  std::vector<double> first_img_t_;

  // decode pipeline
  int decode_threads_;
  size_t queue_cap_;
  std::vector<std::thread> workers_;
  std::atomic<size_t> produce_idx_{0};   // next event a worker will decode
  size_t consume_idx_ = 0;               // next event the caller will see
  std::deque<std::pair<size_t, Frame>> done_;  // decoded, keyed by event index
  std::mutex m_;
  std::condition_variable cv_prod_, cv_cons_;
  std::atomic<bool> stop_{false};
};

}  // namespace rovtio_rosfree
