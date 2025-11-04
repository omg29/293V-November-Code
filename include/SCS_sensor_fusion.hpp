#pragma once
#include "main.h"

namespace SCS {
  // color detection result
  enum class BlockColor { NONE, ALLY, OPPONENT, UNKNOWN };

  // initialize SCS sensor fusion (call in initialize())
  void init();

  // fused heading in degrees (0..360)
  double getFusedHeading();

  // raw imu heading (deg)
  double getImuHeading();

  // raw odom heading (deg) from chassis / odom
  double getOdomHeading();

  // return a debounced/stable block color detection
  BlockColor getStableBlockColor();

  // convenience: process a detected block and return action string or enum
  // e.g., "KEEP", "EJECT", "IGNORE". returns true if action was taken.
  std::string intakeDecision(bool runIntakeMotor = false);

  // optional: run a background small loop for smoothing (call from a task)
  void startBackgroundTask();
  void stopBackgroundTask();
}
