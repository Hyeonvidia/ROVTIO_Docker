// TUM trajectory output plus a diagnostics sidecar. No ROS.
#pragma once

#include <fstream>
#include <iomanip>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rovtio_rosfree {

// TUM: timestamp tx ty tz qx qy qz qw
//
// We write WrWM / qWM directly -- the IMU body pose in ROVIO's gravity-aligned
// world frame. RovioNode publishes the same pose routed through ImuOutputCT,
// which stores qBW = qWM.inverted() and then emits (x,y,z,-w) of it; that is the
// same rotation as qWM, so writing qWM avoids the double sign flip entirely.
class TumWriter {
 public:
  TumWriter() = default;

  bool open(const std::string& path) {
    if (path.empty()) return false;
    f_.open(path);
    if (!f_) return false;
    f_ << "# timestamp tx ty tz qx qy qz qw\n";
    f_ << std::fixed << std::setprecision(9);
    return true;
  }

  bool isOpen() const { return f_.is_open(); }

  void write(double t, const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    if (!f_.is_open()) return;
    f_ << t << ' ' << p.x() << ' ' << p.y() << ' ' << p.z() << ' ' << q.x() << ' ' << q.y()
       << ' ' << q.z() << ' ' << q.w() << '\n';
    ++n_;
  }

  size_t count() const { return n_; }
  void close() { if (f_.is_open()) f_.close(); }

 private:
  std::ofstream f_;
  size_t n_ = 0;
};

// t, position, velocity, accel bias, gyro bias, feature counts, step cost.
class DiagWriter {
 public:
  bool open(const std::string& path) {
    if (path.empty()) return false;
    f_.open(path);
    if (!f_) return false;
    f_ << "#t,px,py,pz,vx,vy,vz,abx,aby,abz,gbx,gby,gbz,nValid,nCam0,nCam1,stepMs\n";
    f_ << std::fixed << std::setprecision(9);
    return true;
  }

  void write(double t, const Eigen::Vector3d& p, const Eigen::Vector3d& v,
             const Eigen::Vector3d& ab, const Eigen::Vector3d& gb, int nValid,
             const int* perCam, int nCam, double stepMs) {
    if (!f_.is_open()) return;
    f_ << t << ',' << p.x() << ',' << p.y() << ',' << p.z() << ',' << v.x() << ',' << v.y()
       << ',' << v.z() << ',' << ab.x() << ',' << ab.y() << ',' << ab.z() << ',' << gb.x()
       << ',' << gb.y() << ',' << gb.z() << ',' << nValid;
    for (int i = 0; i < 2; ++i) f_ << ',' << (i < nCam ? perCam[i] : 0);
    f_ << ',' << stepMs << '\n';
  }

  void close() { if (f_.is_open()) f_.close(); }

 private:
  std::ofstream f_;
};

}  // namespace rovtio_rosfree
