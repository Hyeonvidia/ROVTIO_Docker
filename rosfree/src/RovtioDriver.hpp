// ROS-free port of rovtio::RovioNode's filter-driving logic.
//
// Everything that touched roscpp/cv_bridge/tf is gone; the measurement handling,
// the two-camera queue arbitration and the update stepping are ported literally so
// the numerical behaviour matches the ROS node.
//
// One deliberate deviation from the original, marked [D1] below.
#pragma once

#include <algorithm>
#include <array>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <tuple>
#include <utility>

#include <opencv2/core.hpp>

#include "rovio/FilterStates.hpp"
#include "rovio/RovioFilter.hpp"

namespace rovtio_rosfree {

struct DriverCfg {
  double maxTimeCamInactive = 0.12;      // launch: maxTimeCamInactive
  double maxDelayBeforeDropping = -0.2;  // launch: negative disables dropping
  bool verbose = false;
};

template <typename FILTER>
class RovtioDriver {
 public:
  using mtFilter = FILTER;
  using mtFilterState = typename FILTER::mtFilterState;
  using mtState = typename mtFilterState::mtState;
  using mtPredictionMeas = typename FILTER::mtPrediction::mtMeas;
  using mtImgUpdate = typename std::tuple_element<0, typename FILTER::mtUpdates>::type;
  using mtImgMeas = typename mtImgUpdate::mtMeas;

  RovtioDriver(std::shared_ptr<FILTER> f, const DriverCfg& cfg) : f_(f), cfg_(cfg) {
    for (int i = 0; i < mtState::nCam_; ++i) {
      camActive_[i] = true;   // RovioNode assumes every camera works at startup
      camIgnored_[i] = false;
      framesSinceLastDrop_[i] = -1;  // RovioNode.hpp:396
      tLastImg_[i] = 0.0;
      q_[i];  // create the queue so canAddImage() sees every camera
    }
  }

  void feedImu(double t, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
    predMeas_.template get<mtPredictionMeas::_acc>() = acc;
    predMeas_.template get<mtPredictionMeas::_gyr>() = gyr;
    if (!initialized_) {
      // Single-sample gravity alignment; no stillness check. Valid because every
      // sequence starts stationary.
      f_->resetWithAccelerometer(acc, t);
      for (int i = 0; i < mtState::nCam_; ++i) tLastImg_[i] = t;
      initialized_ = true;
      tInit_ = t;
      return;
    }
    f_->addPredictionMeas(predMeas_, t);
    stepFilter();
  }

  void feedImage(int camID, double t, const cv::Mat& img32f) {
    if (!initialized_) {  // matches RovioNode's init_state_.isInitialized() gate
      ++preInitDropped_;
      return;
    }
    camActive_[camID] = true;
    camIgnored_[camID] = false;
    tLastImg_[camID] = t;  // [D1]

    // [D1] The original compares ros::Time::now() deltas here. Replaying faster
    // than real time makes those deltas ~0, so a camera going silent (a 936 ms
    // Tau2 FFC shutter) never flips camActive_, canAddImage() stays false, and
    // the pipeline stalls for the whole gap. Driving this off the dataset clock
    // keeps maxTimeCamInactive meaningful and makes runs reproducible at any
    // replay rate.
    if (cfg_.maxTimeCamInactive > 0) {
      for (int i = 0; i < mtState::nCam_; ++i) {
        if (i == camID) continue;
        const double dead = tLastImg_[camID] - tLastImg_[i];
        if (dead > cfg_.maxTimeCamInactive) {
          if (!camIgnored_[i] && cfg_.verbose)
            std::printf("[driver] t=%.3f ignoring camera %d, silent for %.3f s\n", t, i, dead);
          camActive_[i] = false;
          camIgnored_[i] = true;
        }
      }
    }
    if (!img32f.empty()) q_[camID].emplace(img32f, t);
  }

  // Feed the remaining queued images once the IMU stream ends, so the tail of the
  // sequence is not silently discarded.
  void flushEndOfStream() {
    for (int guard = 0; guard < 10000; ++guard) {
      bool any = false;
      for (auto& kv : q_)
        if (!kv.second.empty()) any = true;
      if (!any) break;
      for (int i = 0; i < mtState::nCam_; ++i)
        if (q_[i].empty()) camActive_[i] = false;
      stepFilter();
    }
  }

  bool poseUpdated() const { return f_->safe_.t_ > oldSafeTime_; }
  double t() const { return f_->safe_.t_; }
  double tInit() const { return tInit_; }
  const mtState& state() const { return f_->safe_.state_; }
  size_t preInitDropped() const { return preInitDropped_; }
  size_t droppedFrames() const { return droppedFrames_; }
  int nValidFeatures() const {
    int n = 0;
    for (int i = 0; i < mtState::nMax_; ++i)
      if (f_->safe_.fsm_.isValid_[i]) ++n;
    return n;
  }
  void featuresPerCam(int* out) const {
    for (int i = 0; i < mtState::nCam_; ++i) out[i] = 0;
    for (int i = 0; i < mtState::nMax_; ++i)
      if (f_->safe_.fsm_.isValid_[i]) ++out[f_->safe_.fsm_.features_[i].mpCoordinates_->camID_];
  }

 private:
  using Queues = std::map<int, std::queue<std::pair<cv::Mat, double>>>;

  int getOldestCam() const {
    std::pair<int, double> oldest(-1, std::numeric_limits<double>::max());
    for (const auto& e : q_) {
      if (!camActive_[e.first]) continue;
      if (e.second.empty()) continue;
      if (e.second.front().second < oldest.second)
        oldest = std::make_pair(e.first, e.second.front().second);
    }
    return oldest.first;
  }

  bool canAddImage() const {
    for (const auto& e : q_)
      if (e.second.empty() && camActive_[e.first]) return false;
    return true;
  }

  void stepFilter() {
    if (canAddImage()) {
      int camIDOldest = getOldestCam();
      if (camIDOldest >= 0) {
        // maxDelayBeforeDropping is negative in this configuration, so the drop
        // branch never fires; the ++ still runs, and the != 0 test below means the
        // very first round per camera consumes nothing. Kept for bit-fidelity.
        framesSinceLastDrop_[camIDOldest]++;
        if (framesSinceLastDrop_[camIDOldest] != 0 && canAddImage()) {
          auto oldest = q_[camIDOldest].front();
          q_[camIDOldest].pop();
          if (f_->safe_.t_ < oldest.second) {
            auto& aux = imgMeas_.template get<mtImgMeas::_aux>();
            aux.pyr_[camIDOldest].computeFromImage(oldest.first, true);
            aux.activeModality_ = camIDOldest;
            for (int i = 0; i < mtState::nCam_; ++i) aux.camActive_[i] = !camIgnored_[i];
            aux.reset(oldest.second);  // imgTime_; ImgUpdate asserts it equals the key
            f_->template addUpdateMeas<0>(imgMeas_, oldest.second);
          } else {
            ++droppedFrames_;
            if (cfg_.verbose)
              std::printf("[driver] dropped image at %.6f, arrived before safe time %.6f\n",
                          oldest.second, f_->safe_.t_);
          }
        }
      }
    }

    oldSafeTime_ = f_->safe_.t_;
    double lastImageTime;
    if (std::get<0>(f_->updateTimelineTuple_).getLastTime(lastImageTime))
      f_->updateSafe(&lastImageTime);
  }

  std::shared_ptr<FILTER> f_;
  DriverCfg cfg_;
  mtPredictionMeas predMeas_;
  mtImgMeas imgMeas_;  // persistent: each queued measurement deep-copies its pyramids
  Queues q_;
  std::array<bool, mtState::nCam_> camActive_{}, camIgnored_{};
  std::array<double, mtState::nCam_> tLastImg_{};
  std::array<int, mtState::nCam_> framesSinceLastDrop_{};
  bool initialized_ = false;
  double tInit_ = 0.0;
  double oldSafeTime_ = 0.0;
  size_t preInitDropped_ = 0, droppedFrames_ = 0;
};

}  // namespace rovtio_rosfree
