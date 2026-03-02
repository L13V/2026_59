// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ramtech.frc2026.subsystems.shooter;

import org.littletonrobotics.junction.Logger;
import org.ramtech.frc2026.Constants.Offsets;
import org.ramtech.frc2026.Constants.TargetPoses;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.ramtech.frc2026.RobotState;

public class ShotCalculator {
  private static ShotCalculator instance = new ShotCalculator();

  public static ShotCalculator getInstance() {
    return instance;
  }

  public record ShotParameters(
      boolean isValid,
      double hoodAngle,
      double flywheelVelocity,
      double towerVelocity,
      double turretAngle) {
  }

  // 'volatile' ensures the 20ms thread always sees the most recent write
  // from the 5ms thread without the overhead of heavy synchronization.
  private volatile ShotParameters latest = new ShotParameters(false, 0, 0, 0, 0);

  /*
   * Turret
   */

  public double getAngleToHub() {
    Pose3d robotpose = new Pose3d(RobotState.getInstance().getRobotPose());
    Pose3d turretpose = robotpose.transformBy(Offsets.turretOffset);

    // x and y translation to the center of the hub
    var translationToHub = TargetPoses.hub.getTranslation().minus(turretpose.getTranslation());
    // top-down angle to hub
    Rotation2d fieldAngleToHub = new Rotation2d(translationToHub.getX(), translationToHub.getY());
    // turret relative angle (center of turret to the hub)
    Rotation2d robotRelativeAngle = fieldAngleToHub.minus(RobotState.getInstance().getRobotPose().getRotation());
    double targetDegrees = robotRelativeAngle.getDegrees();
    return targetDegrees;
  }

  public void update(double loopTime) {
    double turretAngle = getAngleToHub();
    boolean isValid = true;
    latest = new ShotParameters(
        isValid,
        35.0, // Interpolated Hood
        3000.0, // Interpolated Flywheel
        10.0,
        turretAngle);
  }

  public ShotParameters getLatest() {
    return latest;
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
