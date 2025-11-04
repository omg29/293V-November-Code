#include "SCS_adaptive_pid.hpp"
#include <fstream>
#include <cmath>
#include <algorithm>

static double kP = 20.0;
static double kI = 0.0;
static double kD = 100.0;

const std::string PID_SAVE_PATH = "/usd/ai_pid.txt";

constexpr double LEARNING_RATE_P = 0.05;
constexpr double LEARNING_RATE_D = 0.05;

constexpr double KP_MIN = 5.0, KP_MAX = 60.0;
constexpr double KD_MIN = 5.0, KD_MAX = 200.0;
constexpr double TOLERANCE_IN = 0.5;

void ai_load_pid() {
  std::ifstream f(PID_SAVE_PATH);
  if (f.is_open()) f >> kP >> kI >> kD;
  f.close();
  chassis.pid_drive_constants_set(kP, kI, kD);
}

void ai_save_pid() {
  std::ofstream f(PID_SAVE_PATH);
  if (f.is_open()) f << kP << " " << kI << " " << kD << "\n";
  f.close();
}

// Run a drive and adjust PID automatically
void ai_auto_tune_drive(double targetInches) {
  ai_load_pid();

  double start = chassis.drive_imu_get();
  chassis.pid_drive_set(targetInches, 100);
  chassis.pid_wait();

  double end = chassis.drive_imu_get();
  double error = (start + targetInches) - end;  // + means undershoot, - means overshoot
  double overshoot = fabs(end - (start + targetInches));

  bool undershot = (error > TOLERANCE_IN);
  bool overshot = (error < -TOLERANCE_IN);

  if (overshot) {
    kP = std::clamp(kP * (1.0 - LEARNING_RATE_P), KP_MIN, KP_MAX);
    kD = std::clamp(kD * (1.0 + LEARNING_RATE_D), KD_MIN, KD_MAX);
  } else if (undershot) {
    kP = std::clamp(kP * (1.0 + LEARNING_RATE_P), KP_MIN, KP_MAX);
    kD = std::clamp(kD * (1.0 - LEARNING_RATE_D / 2), KD_MIN, KD_MAX);
  }

  chassis.pid_drive_constants_set(kP, kI, kD);
  ai_save_pid();

  std::printf("[AI PID] final error=%.3f, new PID: P=%.2f I=%.2f D=%.2f\n",
              error, kP, kI, kD);
}
