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

  public static class ShotParameters {
    private boolean isValid;
    private double hoodAngle;
    private double flywheelVelocity;
    private double towerVelocity;
    private double turretAngle;

    public ShotParameters(
        boolean isValid,
        double hoodAngle,
        double flywheelVelocity,
        double towerVelocity,
        double turretAngle) {
      this.isValid = isValid;
      this.hoodAngle = hoodAngle;
      this.flywheelVelocity = flywheelVelocity;
      this.towerVelocity = towerVelocity;
      this.turretAngle = turretAngle;
    }

    public void set(
        boolean isValid,
        double hoodAngle,
        double flywheelVelocity,
        double towerVelocity,
        double turretAngle) {
      this.isValid = isValid;
      this.hoodAngle = hoodAngle;
      this.flywheelVelocity = flywheelVelocity;
      this.towerVelocity = towerVelocity;
      this.turretAngle = turretAngle;
    }

    public boolean isValid() {
      return isValid;
    }

    public double hoodAngle() {
      return hoodAngle;
    }

    public double flywheelVelocity() {
      return flywheelVelocity;
    }

    public double towerVelocity() {
      return towerVelocity;
    }

    public double turretAngle() {
      return turretAngle;
    }

    @Override
    public String toString() {
      return "ShotParameters{"
          + "isValid="
          + isValid
          + ", hoodAngle="
          + hoodAngle
          + ", flywheelVelocity="
          + flywheelVelocity
          + ", towerVelocity="
          + towerVelocity
          + ", turretAngle="
          + turretAngle
          + '}';
    }
  }

  private final ShotParameters params1 = new ShotParameters(false, 0, 0, 0, 0);
  private final ShotParameters params2 = new ShotParameters(false, 0, 0, 0, 0);

  private final AtomicReference<ShotParameters> latest = new AtomicReference<>(params1);

  public void update(double loopTime) {
    ShotParameters newParams = latest.get() == params1 ? params2 : params1;
    newParams.set(false, 0.0, 0.0, 0.0, 0.0);
    latest.set(newParams);
  }

  public ShotParameters getLatest() {
    return latest.get();
  }

  public void publishShotParameters() {
    var params = getLatest();
    Logger.recordOutput("ShotCalculator/ShotParameters/IsValid", params.isValid());
    Logger.recordOutput("ShotCalculator/ShotParameters/HoodAngle", params.hoodAngle());
    Logger.recordOutput(
        "ShotCalculator/ShotParameters/FlywheelVelocity", params.flywheelVelocity());
    Logger.recordOutput("ShotCalculator/ShotParameters/TowerVelocity", params.towerVelocity());
    Logger.recordOutput("ShotCalculator/ShotParameters/TurretAngle", params.turretAngle());
  }
}
