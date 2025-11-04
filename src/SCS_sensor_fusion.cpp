#include "SCS_sensor_fusion.hpp"
#include <atomic>
#include <thread>
#include <vector>
#include <numeric>

// ----- Hardware ports - change to your actual ports -----
static constexpr int IMU_PORT = 8;         // example from your template
static constexpr int OPTICAL_PORT = 1;     // change to the ADI/port the optical sensor is on

// ----- Team color setting (set this to your color) -----
static constexpr bool TEAM_IS_RED = true; // set false if you are blue

// ----- Fusion parameters -----
static constexpr double HEADING_ALPHA = 0.85; // weight for IMU in complementary filter (0..1)
                                             // higher => trust IMU more; lower => trust odom more

// ----- Optical smoothing / debounce -----
static constexpr int COLOR_WINDOW = 5; // moving average window (frames)
static constexpr int COLOR_DEBOUNCE_COUNT = 4; // how many consistent frames to accept

// ----- PROS sensor objects -----
static pros::Imu imu(7);
static pros::Optical optical(8); // if your optical sensor uses ADI or different class, replace accordingly

// ----- internal buffers & state -----
static std::vector<double> rbuf(COLOR_WINDOW, 0.0);
static std::vector<double> gbuf(COLOR_WINDOW, 0.0);
static std::vector<double> bbuf(COLOR_WINDOW, 0.0);
static int bufIndex = 0;
static std::atomic<int> stableColorCounter{0};
static std::atomic<SCS::BlockColor> stableColor{SCS::BlockColor::NONE};

static pros::Task* bgTask = nullptr;
static bool bgTaskRunning = false;

// helper: normalize heading to [0,360)
static double norm360(double deg) {
  double d = fmod(deg, 360.0);
  if (d < 0) d += 360.0;
  return d;
}

// read raw optical rgb - returns tuple (r,g,b) as doubles
static void readOpticalRGB(double &r, double &g, double &b) {
  // PROS Optical API: optical.get_rgb()
  // If your sensor is different, replace with appropriate calls (e.g., optical.get_hsv())
  pros::c::optical_rgb_s_t rgb = optical.get_rgb();
  r = static_cast<double>(rgb.red);
  g = static_cast<double>(rgb.green);
  b = static_cast<double>(rgb.blue);
}

// convert averaged rgb to BlockColor
static SCS::BlockColor rgbToColor(double r, double g, double b) {
  // Basic heuristic:
  // - If total intensity is too low => NONE (no block / no object)
  double total = r + g + b;
  if (total < 60.0) return SCS::BlockColor::NONE; // tune threshold

  // compute normalized red/blue dominance
  double redScore = r / total;
  double blueScore = b / total;

  // thresholds tuned for VEX colored tiles/balls; may need to tweak
  if (redScore > 0.48 && redScore - blueScore > 0.08) {
    return TEAM_IS_RED ? SCS::BlockColor::ALLY : SCS::BlockColor::OPPONENT;
  } else if (blueScore > 0.48 && blueScore - redScore > 0.08) {
    return TEAM_IS_RED ? SCS::BlockColor::OPPONENT : SCS::BlockColor::ALLY;
  } else {
    return SCS::BlockColor::UNKNOWN;
  }
}

// background task: reads optical repeatedly, smooths, and debounces
static void backgroundLoop() {
  double r, g, b;
  readOpticalRGB(r, g, b);

  // update buffers and compute averages (same as before)
  rbuf[bufIndex] = r;
  gbuf[bufIndex] = g;
  bbuf[bufIndex] = b;
  bufIndex = (bufIndex + 1) % COLOR_WINDOW;

  double ravg = std::accumulate(rbuf.begin(), rbuf.end(), 0.0) / rbuf.size();
  double gavg = std::accumulate(gbuf.begin(), gbuf.end(), 0.0) / gbuf.size();
  double bavg = std::accumulate(bbuf.begin(), bbuf.end(), 0.0) / bbuf.size();

  SCS::BlockColor cur = rgbToColor(ravg, gavg, bavg);

  static SCS::BlockColor lastDetected = SCS::BlockColor::NONE;
  if (cur == lastDetected && cur != SCS::BlockColor::NONE) {
    stableColorCounter++;
  } else {
    stableColorCounter = 0;
  }
  lastDetected = cur;

  if (stableColorCounter >= COLOR_DEBOUNCE_COUNT) {
    stableColor.store(cur);
  } else if (cur == SCS::BlockColor::NONE) {
    stableColor.store(SCS::BlockColor::NONE);
  }
}


// Public API implementations
namespace SCS {

  void init() {
    // calibrate imu if needed
    imu.reset();
    pros::delay(50);

    // zero buffers
    std::fill(rbuf.begin(), rbuf.end(), 0.0);
    std::fill(gbuf.begin(), gbuf.end(), 0.0);
    std::fill(bbuf.begin(), bbuf.end(), 0.0);

    // start background thread
    startBackgroundTask();
  }

  double getImuHeading() {
    // imu.get_rotation() returns degrees in PROS (0..360). check your API if different.
    double deg = imu.get_rotation();
    return norm360(deg);
  }

  double getOdomHeading() {
    // use EZ-Template odom theta (in degrees)
    double odomDeg = chassis.odom_theta_get();
    return norm360(odomDeg);
  }

  double getFusedHeading() {
    // complementary filter
    double imuH = getImuHeading();
    double odomH = getOdomHeading();

    // compute shortest difference for wrap-around
    double diff = imuH - odomH;
    // normalize diff to -180..180
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    double fused = odomH + HEADING_ALPHA * diff;
    return norm360(fused);
  }

  BlockColor getStableBlockColor() {
    return stableColor.load();
  }

  std::string intakeDecision(bool runIntakeMotor) {
    BlockColor c = getStableBlockColor();

    if (c == BlockColor::NONE) {
      return "NONE";
    } else if (c == BlockColor::ALLY) {
      if (runIntakeMotor) {
        // keep running intake (user must implement intake motor call)
        // e.g., intake.move(127) in your intake module
      }
      return "KEEP";
    } else if (c == BlockColor::OPPONENT) {
      // eject block
      // e.g., intake.move(-127) for a short burst
      return "EJECT";
    } else {
      return "UNKNOWN";
    }
  }

    void startBackgroundTask() {
        if (bgTaskRunning) return;
        bgTaskRunning = true;

        // Start a PROS task instead of std::thread
        bgTask = new pros::Task([] {
            pros::delay(100);
            while (bgTaskRunning) {
            backgroundLoop();
            pros::delay(10); // small delay to prevent CPU overload
            }
        });
    }

void stopBackgroundTask() {
  bgTaskRunning = false;
  pros::delay(50);
  if (bgTask != nullptr) {
    delete bgTask;
    bgTask = nullptr;
  }
}
}
