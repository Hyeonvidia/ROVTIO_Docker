// rovtio_player -- runs ROVTIO on an extracted (ROS-free) ROVTIO dataset sequence,
// optionally with the built-in OpenGL scene and the 2D tracker overlays.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#ifdef ROVTIO_WITH_HIGHGUI
#include <opencv2/highgui.hpp>
#endif

#include "rovio/RovioFilter.hpp"
#ifdef MAKE_SCENE
#include "rovio/RovioScene.hpp"
#endif

#include "DatasetPlayer.hpp"
#include "RovtioDriver.hpp"
#include "TumWriter.hpp"

typedef rovio::RovioFilter<rovio::FilterState<ROVIO_NMAXFEATURE, ROVIO_NLEVELS,
                                              ROVIO_PATCHSIZE, ROVIO_NCAM, ROVIO_NPOSE>>
    mtFilter;

using namespace rovtio_rosfree;

namespace {

struct Cfg {
  std::string seq;
  std::string info = "/opt/rovtio/cfg/rovtio_rosfree.info";
  std::string cam_yaml[2];
  std::string shaders = "/opt/rovtio/shaders";
  std::string tum, diag;
  std::string gui = "none";  // none | trackers | all
  double rate = 0.0;         // 0 = as fast as possible
  double viewer_hz = 15.0;
  double cam_offset[2] = {0.0, -0.02414188675155223};
  int cams = -1;  // -1 = all compiled-in cameras
  int decode_threads = 3;
  bool dry_run = false;
  bool verbose = false;
  double max_seconds = 0.0;
};

const char* kUsage =
    "rovtio_player --seq <dir> [options]\n"
    "  --seq DIR            extracted sequence dir (contains vn100_imu.csv)\n"
    "  --info FILE          filter .info file\n"
    "  --cam0-yaml FILE     visual camera calibration\n"
    "  --cam1-yaml FILE     thermal camera calibration\n"
    "  --cams N             use only the first N cameras (default: all)\n"
    "  --cam1-offset S      thermal time offset, t_imu = t_cam + S\n"
    "  --tum FILE           write TUM trajectory\n"
    "  --diag FILE          write diagnostics csv\n"
    "  --gui none|trackers|all\n"
    "  --rate R             0 = unlimited (default), 1.0 = real time\n"
    "  --viewer-hz H        3D redraw cap (default 15)\n"
    "  --shaders DIR        shader dir (default /opt/rovtio/shaders)\n"
    "  --max-seconds S      stop after S seconds of data\n"
    "  --threads N          png decode threads (default 3)\n"
    "  --dry-run            load config, print calibration, exit\n"
    "  --verbose\n";

bool argIs(const char* a, const char* n) { return std::strcmp(a, n) == 0; }

}  // namespace

int main(int argc, char** argv) {
  Cfg cfg;
  for (int i = 1; i < argc; ++i) {
    const char* a = argv[i];
    auto need = [&](const char* name) -> const char* {
      if (i + 1 >= argc) { std::fprintf(stderr, "%s needs a value\n", name); std::exit(2); }
      return argv[++i];
    };
    if (argIs(a, "--seq")) cfg.seq = need(a);
    else if (argIs(a, "--info")) cfg.info = need(a);
    else if (argIs(a, "--cam0-yaml")) cfg.cam_yaml[0] = need(a);
    else if (argIs(a, "--cam1-yaml")) cfg.cam_yaml[1] = need(a);
    else if (argIs(a, "--cams")) cfg.cams = std::atoi(need(a));
    else if (argIs(a, "--cam1-offset")) cfg.cam_offset[1] = std::atof(need(a));
    else if (argIs(a, "--tum")) cfg.tum = need(a);
    else if (argIs(a, "--diag")) cfg.diag = need(a);
    else if (argIs(a, "--gui")) cfg.gui = need(a);
    else if (argIs(a, "--rate")) cfg.rate = std::atof(need(a));
    else if (argIs(a, "--viewer-hz")) cfg.viewer_hz = std::atof(need(a));
    else if (argIs(a, "--shaders")) cfg.shaders = need(a);
    else if (argIs(a, "--max-seconds")) cfg.max_seconds = std::atof(need(a));
    else if (argIs(a, "--threads")) cfg.decode_threads = std::atoi(need(a));
    else if (argIs(a, "--dry-run")) cfg.dry_run = true;
    else if (argIs(a, "--verbose")) cfg.verbose = true;
    else if (argIs(a, "-h") || argIs(a, "--help")) { std::printf("%s", kUsage); return 0; }
    else { std::fprintf(stderr, "unknown option %s\n\n%s", a, kUsage); return 2; }
  }
  const int nCam = (cfg.cams > 0 && cfg.cams < mtFilter::mtFilterState::mtState::nCam_)
                       ? cfg.cams
                       : mtFilter::mtFilterState::mtState::nCam_;
  if (cfg.seq.empty() && !cfg.dry_run) { std::fprintf(stderr, "%s", kUsage); return 2; }

  // ---- filter -------------------------------------------------------------
  auto filter = std::make_shared<mtFilter>();
  filter->readFromInfo(cfg.info);  // calls refreshProperties() internally
  for (int i = 0; i < mtFilter::mtFilterState::mtState::nCam_; ++i)
    if (i < 2 && !cfg.cam_yaml[i].empty()) filter->cameraCalibrationFile_[i] = cfg.cam_yaml[i];
  filter->refreshProperties();  // must follow any override

  std::printf("ROVTIO ROS-free player\n");
  std::printf("  compiled: nCam=%d nMaxFeature=%d nLevels=%d patchSize=%d\n",
              (int)mtFilter::mtFilterState::mtState::nCam_, ROVIO_NMAXFEATURE, ROVIO_NLEVELS,
              ROVIO_PATCHSIZE);
  std::printf("  using   : %d camera(s)\n  info    : %s\n", nCam, cfg.info.c_str());
  bool calib_ok = true;
  for (int i = 0; i < nCam; ++i) {
    const auto& cam = filter->multiCamera_.cameras_[i];
    std::printf("  cam%d K=[%.4f %.4f %.4f %.4f] model=%d D=[%.6f %.6f %.6f %.6f]\n", i,
                cam.K_(0, 0), cam.K_(1, 1), cam.K_(0, 2), cam.K_(1, 2), (int)cam.type_,
                cam.k1_, cam.k2_, cam.k3_, cam.k4_);
    if (!(cam.K_(0, 0) > 100.0)) {
      std::fprintf(stderr,
                   "  !! cam%d focal length is %.4f -- the calibration was NOT loaded.\n"
                   "     Set Camera%d.CalibrationFile in the .info or pass --cam%d-yaml.\n",
                   i, cam.K_(0, 0), i, i);
      calib_ok = false;
    }
  }
  if (!calib_ok) return 3;
  if (cfg.dry_run) { std::printf("dry run: configuration OK\n"); return 0; }

  // ---- dataset ------------------------------------------------------------
  std::vector<CamSpec> cams;
  cams.push_back({cfg.seq + "/cam_blackfly_cam_blackfly", cfg.cam_offset[0], false});
  if (nCam > 1)
    cams.push_back({cfg.seq + "/tau_nodelet_thermal_image", cfg.cam_offset[1], true});

  DatasetPlayer player(cfg.seq, cfg.seq + "/vn100_imu.csv", cams, cfg.decode_threads);
  try {
    player.load();
  } catch (const std::exception& e) {
    std::fprintf(stderr, "dataset error: %s\n", e.what());
    return 4;
  }
  std::printf("  events  : %zu (imu %zu", player.size(), player.nImu());
  for (int i = 0; i < nCam; ++i) std::printf(", cam%d %zu", i, player.nImg(i));
  std::printf(")\n  span    : %.3f s\n", player.t1() - player.t0());
  for (int i = 0; i < nCam; ++i)
    std::printf("  cam%d starts %+.3f s relative to the first IMU sample\n", i,
                player.firstImgTime(i) - player.firstImuTime());
  player.start();

  // ---- driver / output ----------------------------------------------------
  DriverCfg dcfg;
  dcfg.verbose = cfg.verbose;
  RovtioDriver<mtFilter> driver(filter, dcfg);
  TumWriter tum;
  DiagWriter diag;
  if (!cfg.tum.empty() && !tum.open(cfg.tum))
    std::fprintf(stderr, "cannot write %s\n", cfg.tum.c_str());
  if (!cfg.diag.empty()) diag.open(cfg.diag);

  const bool want_trackers = (cfg.gui == "trackers" || cfg.gui == "all");
  const bool want_scene = (cfg.gui == "all");
  auto& imgUpd = std::get<0>(filter->mUpdates_);
  if (want_trackers) imgUpd.doFrameVisualisation_ = true;

#ifdef ROVTIO_WITH_HIGHGUI
  // highgui puts every window at +0+0 and there is no window manager in the
  // container, so without this the tracker windows sit exactly on top of each
  // other and only the last one created is ever visible.
  if (want_trackers) {
    for (int i = 0; i < nCam; ++i) {
      const std::string w = "Tracker" + std::to_string(i);
      cv::namedWindow(w, cv::WINDOW_NORMAL);
      cv::resizeWindow(w, 760, 580);
      cv::moveWindow(w, want_scene ? 1400 : 20, 20 + i * 620);
    }
  }
#endif

#ifdef MAKE_SCENE
  static rovio::RovioScene<mtFilter> gScene;
  std::shared_ptr<rovio::SceneObject> trail;
  if (want_scene) {
    gScene.initScene(argc, argv, cfg.shaders + "/shader.vs", cfg.shaders + "/shader.fs", filter);
    trail = gScene.mScene.addSceneObject();
    trail->makeLine();
    trail->mode_ = GL_LINE_STRIP;
    trail->lineWidth_ = 2.0f;
    trail->setColorFull(Eigen::Vector4f(0.1f, 0.3f, 1.0f, 1.0f));
  }
#else
  if (want_scene) {
    std::fprintf(stderr, "built without MAKE_SCENE; falling back to --gui trackers\n");
    imgUpd.doFrameVisualisation_ = true;
  }
#endif

  // ---- replay loop --------------------------------------------------------
  using clk = std::chrono::steady_clock;
  const auto wall0 = clk::now();
  double data0 = -1.0, lastDraw = -1.0;
  size_t nPose = 0, nTrail = 0;
  Event ev;
  Frame fr;
  std::printf("  replay  : rate=%s gui=%s\n\n",
              cfg.rate > 0 ? std::to_string(cfg.rate).c_str() : "unlimited", cfg.gui.c_str());

  while (player.next(ev, fr)) {
    if (data0 < 0) data0 = ev.t;
    if (cfg.max_seconds > 0 && ev.t - data0 > cfg.max_seconds) break;

    if (cfg.rate > 0) {  // pace to wall clock
      const double target = (ev.t - data0) / cfg.rate;
      for (;;) {
        const double elapsed = std::chrono::duration<double>(clk::now() - wall0).count();
        if (elapsed >= target) break;
        std::this_thread::sleep_for(std::chrono::duration<double>(
            std::min(0.005, target - elapsed)));
      }
    }

    const auto tStep = clk::now();
    if (ev.kind == Event::kImu) {
      driver.feedImu(ev.t, ev.acc, ev.gyr);
    } else {
      driver.feedImage(ev.camID, ev.t, fr.img);
      continue;  // images never step the filter; only IMU does
    }
    const double stepMs = std::chrono::duration<double, std::milli>(clk::now() - tStep).count();

    if (driver.poseUpdated()) {
      const auto& st = driver.state();
      const Eigen::Vector3d p = st.WrWM();
      const auto& kq = st.qWM();
      const Eigen::Quaterniond q(kq.w(), kq.x(), kq.y(), kq.z());
      tum.write(driver.t(), p, q);
      int perCam[2] = {0, 0};
      driver.featuresPerCam(perCam);
      diag.write(driver.t(), p, st.MvM(), st.acb(), st.gyb(), driver.nValidFeatures(), perCam,
                 nCam, stepMs);
      ++nPose;
#ifdef MAKE_SCENE
      if (trail && (nPose % 10 == 0) && nTrail < 5000) {  // prolonge() re-uploads the VBO
        trail->prolonge(p.cast<float>());
        ++nTrail;
      }
#endif
      if (nPose % 500 == 0)
        std::printf("  t=%9.3f  pos=[%7.3f %7.3f %7.3f]  feat=%2d  %.1f ms/step\n",
                    driver.t() - driver.tInit(), p.x(), p.y(), p.z(), driver.nValidFeatures(),
                    stepMs);
    }

    // ---- visualisation --------------------------------------------------
    const double nowS = std::chrono::duration<double>(clk::now() - wall0).count();
    if (nowS - lastDraw > 1.0 / cfg.viewer_hz) {
      lastDraw = nowS;
#ifdef MAKE_SCENE
      if (want_scene) {
        glutMainLoopEvent();  // freeglut: pump X events and return
        gScene.drawScene(filter->safe_);
        gScene.mScene.RenderSceneCB();
      }
#endif
#ifdef ROVTIO_WITH_HIGHGUI
      if (want_trackers) {
        for (int i = 0; i < nCam; ++i)
          if (!filter->safe_.img_[i].empty())
            cv::imshow("Tracker" + std::to_string(i), filter->safe_.img_[i]);
        if (cv::waitKey(1) == 27) break;
      }
#endif
    }
  }

  driver.flushEndOfStream();
  std::printf("\ndone: %zu poses written", nPose);
  if (!cfg.tum.empty()) std::printf(" -> %s", cfg.tum.c_str());
  std::printf("\n  pre-init images dropped: %zu, late images dropped: %zu\n",
              driver.preInitDropped(), driver.droppedFrames());
  tum.close();
  diag.close();
  return 0;
}
