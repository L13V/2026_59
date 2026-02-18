// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.ramtech.frc2026.util;

import java.util.ArrayList;
import java.util.List;

/**
 * A standard subsystem that includes an extra periodic callback which runs after the command
 * scheduler. Allows outputs to be published after all other periodic code has finished.
 */
public abstract class ShooterSubsystem extends FullSubsystem {
  private static List<ShooterSubsystem> instances = new ArrayList<>();

  public ShooterSubsystem() {
    super();
    instances.add(this);
  }

  public ShooterSubsystem(String name) {
    super(name);
    instances.add(this);
  }

  public abstract void shooterPeriodic(double dt);

  public static void runAllShooterMotorPeriodics(double dt) {
    for (ShooterSubsystem instance : instances) {
      instance.shooterPeriodic(dt);
    }
  }
}
