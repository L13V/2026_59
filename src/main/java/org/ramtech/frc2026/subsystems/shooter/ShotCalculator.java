// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ramtech.frc2026.subsystems.shooter;

import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ShotCalculator {
  private static ShotCalculator instance = new ShotCalculator();

  public static ShotCalculator getInstance() {
    return instance;
  }

  public record ShotParameters(
      boolean isValid, double hoodAngle, double flywheelVelocity, double towerVelocity) {}

  private final AtomicReference<ShotParameters> latest =
      new AtomicReference<>(new ShotParameters(false, 0.0, 0.0, 0.0));

  public void update(double loopTime) {
    ShotParameters params = new ShotParameters(false, 0.0, 0.0, 0.0);
    latest.set(params);
  }

  public ShotParameters getLatest() {
    return latest.get();
  }

  public void publishShotParameters() {
    var params = getLatest();
    Logger.recordOutput("ShotCalculator/ShotParameters", params);
  }
}
