#include "DatasetPlayer.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace rovtio_rosfree {
namespace {

std::vector<std::string> split(const std::string& s, char d) {
  std::vector<std::string> out;
  std::string cur;
  std::istringstream is(s);
  while (std::getline(is, cur, d)) out.push_back(cur);
  return out;
}

}  // namespace

DatasetPlayer::DatasetPlayer(const std::string& seq_dir, const std::string& imu_csv,
                             const std::vector<CamSpec>& cams, int decode_threads,
                             size_t queue_cap)
    : seq_dir_(seq_dir),
      imu_csv_(imu_csv),
      cams_(cams),
      decode_threads_(decode_threads),
      queue_cap_(queue_cap) {
  n_img_.assign(cams_.size(), 0);
  first_img_t_.assign(cams_.size(), 0.0);
}

DatasetPlayer::~DatasetPlayer() {
  stop_ = true;
  cv_prod_.notify_all();
  cv_cons_.notify_all();
  for (auto& t : workers_)
    if (t.joinable()) t.join();
}

void DatasetPlayer::load() {
  events_.clear();

  // ---- IMU: #timestamp [ns], w_x,w_y,w_z [rad/s], a_x,a_y,a_z [m/s^2], q_* ----
  {
    std::ifstream f(imu_csv_);
    if (!f) throw std::runtime_error("cannot open IMU csv: " + imu_csv_);
    std::string line;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#') continue;
      auto p = split(line, ',');
      if (p.size() < 7) throw std::runtime_error("short IMU row: " + line);
      Event e;
      e.kind = Event::kImu;
      e.t = std::stod(p[0]) * 1e-9;
      e.gyr << std::stod(p[1]), std::stod(p[2]), std::stod(p[3]);
      e.acc << std::stod(p[4]), std::stod(p[5]), std::stod(p[6]);
      events_.push_back(std::move(e));
    }
    n_imu_ = events_.size();
    if (n_imu_ == 0) throw std::runtime_error("no IMU samples in " + imu_csv_);
    first_imu_t_ = events_.front().t;
  }

  // ---- images: #timestamp [ns], filename, bag_time [ns] ----
  // Field 0 is the sensor header stamp -- the same field RovioNode reads.
  // Field 2 is bag receive time (3-14 ms later) and must not be used.
  for (size_t c = 0; c < cams_.size(); ++c) {
    const std::string csv = cams_[c].dir + "/data.csv";
    std::ifstream f(csv);
    if (!f) throw std::runtime_error("cannot open image csv: " + csv);
    std::string line;
    size_t n = 0;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#') continue;
      auto p = split(line, ',');
      if (p.size() < 2) throw std::runtime_error("short image row: " + line);
      Event e;
      e.kind = Event::kImg;
      e.camID = (int)c;
      e.t = std::stod(p[0]) * 1e-9 + cams_[c].offset;
      e.path = cams_[c].dir + "/data/" + p[1];
      if (n == 0) first_img_t_[c] = e.t;
      events_.push_back(std::move(e));
      ++n;
    }
    n_img_[c] = n;
    if (n == 0) throw std::runtime_error("no images listed in " + csv);
  }

  // Stable sort by time; on an exact tie the IMU sample goes first so the filter
  // has propagated to that instant before the update is applied.
  std::stable_sort(events_.begin(), events_.end(), [](const Event& a, const Event& b) {
    if (a.t != b.t) return a.t < b.t;
    return (int)a.kind < (int)b.kind;
  });
}

cv::Mat DatasetPlayer::decodeVisual(const std::string& path) {
  cv::Mat bgr = cv::imread(path, cv::IMREAD_COLOR);
  if (bgr.empty()) throw std::runtime_error("failed to read visual image: " + path);
  CV_Assert(bgr.type() == CV_8UC3);
  cv::Mat gray, f32;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  gray.convertTo(f32, CV_32FC1);
  return f32;
}

cv::Mat DatasetPlayer::decodeThermal(const std::string& path) {
  // IMREAD_UNCHANGED, never IMREAD_GRAYSCALE: the latter right-shifts to 8 bit and
  // both destroys thermal texture and silently disables ImgUpdate's own 16-bit
  // equalisation (which triggers on max > 255).
  cv::Mat raw = cv::imread(path, cv::IMREAD_UNCHANGED);
  if (raw.empty()) throw std::runtime_error("failed to read thermal image: " + path);
  if (raw.type() != CV_16UC1)
    throw std::runtime_error("thermal image is not 16-bit single channel: " + path +
                             " (type=" + std::to_string(raw.type()) + ")");
  cv::Mat f32;
  raw.convertTo(f32, CV_32FC1);  // no alpha/beta: keep raw counts
  return f32;
}

void DatasetPlayer::worker() {
  while (!stop_) {
    const size_t idx = produce_idx_.fetch_add(1);
    if (idx >= events_.size()) return;
    const Event& e = events_[idx];
    if (e.kind != Event::kImg) continue;  // IMU needs no decoding

    // Bound memory by how far AHEAD of the consumer we are, not by queue length.
    // Bounding by length can deadlock: the consumer blocks on the lowest pending
    // index while faster workers fill the queue with higher ones, and the worker
    // holding that lowest index can then never push.
    {
      std::unique_lock<std::mutex> lk(m_);
      cv_prod_.wait(lk, [&] { return stop_ || idx < consume_idx_ + queue_cap_; });
      if (stop_) return;
    }

    Frame fr;
    fr.camID = e.camID;
    fr.t = e.t;
    try {
      fr.img = cams_[e.camID].thermal ? decodeThermal(e.path) : decodeVisual(e.path);
    } catch (const std::exception& ex) {
      std::fprintf(stderr, "[player] %s\n", ex.what());
      fr.img = cv::Mat();
    }

    std::unique_lock<std::mutex> lk(m_);
    done_.emplace_back(idx, std::move(fr));
    cv_cons_.notify_all();
  }
}

void DatasetPlayer::start() {
  for (int i = 0; i < decode_threads_; ++i) workers_.emplace_back([this] { worker(); });
}

bool DatasetPlayer::next(Event& ev, Frame& frame) {
  size_t idx;
  {
    std::lock_guard<std::mutex> lk(m_);
    if (consume_idx_ >= events_.size()) return false;
    idx = consume_idx_++;
  }
  cv_prod_.notify_all();  // the window moved; workers may proceed
  ev = events_[idx];
  if (ev.kind != Event::kImg) return true;

  std::unique_lock<std::mutex> lk(m_);
  cv_cons_.wait(lk, [&] {
    if (stop_) return true;
    for (const auto& d : done_)
      if (d.first == idx) return true;
    return false;
  });
  for (auto it = done_.begin(); it != done_.end(); ++it) {
    if (it->first == idx) {
      frame = std::move(it->second);
      done_.erase(it);
      break;
    }
  }
  return true;
}

}  // namespace rovtio_rosfree
