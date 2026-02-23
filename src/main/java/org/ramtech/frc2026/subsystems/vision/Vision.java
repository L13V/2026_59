// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.ramtech.frc2026.subsystems.vision;

import static org.ramtech.frc2026.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.ramtech.frc2026.subsystems.vision.VisionIO.PoseObservationType;

public class Vision extends SubsystemBase {
  private static final double LOG_PERIOD_SEC = 0.1;

  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private double lastLogTimestamp = Double.NEGATIVE_INFINITY;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    boolean logThisCycle = (Timer.getFPGATimestamp() - lastLogTimestamp) >= LOG_PERIOD_SEC;
    if (logThisCycle) {
      lastLogTimestamp = Timer.getFPGATimestamp();
    }

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = logThisCycle ? new ArrayList<>() : null;
    List<Pose3d> allRobotPoses = logThisCycle ? new ArrayList<>() : null;
    List<Pose3d> allRobotPosesAccepted = logThisCycle ? new ArrayList<>() : null;
    List<Pose3d> allRobotPosesRejected = logThisCycle ? new ArrayList<>() : null;

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = logThisCycle ? new ArrayList<>() : null;
      List<Pose3d> robotPoses = logThisCycle ? new ArrayList<>() : null;
      List<Pose3d> robotPosesAccepted = logThisCycle ? new ArrayList<>() : null;
      List<Pose3d> robotPosesRejected = logThisCycle ? new ArrayList<>() : null;

      // Add tag poses
      if (logThisCycle) {
        for (int tagId : inputs[cameraIndex].tagIds) {
          var tagPose = aprilTagLayout.getTagPose(tagId);
          if (tagPose.isPresent()) {
            tagPoses.add(tagPose.get());
          }
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        if (logThisCycle) {
          robotPoses.add(observation.pose());
          if (rejectPose) {
            robotPosesRejected.add(observation.pose());
          } else {
            robotPosesAccepted.add(observation.pose());
          }
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      if (logThisCycle) {
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
            tagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
            robotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
            robotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
            robotPosesRejected.toArray(new Pose3d[0]));
        allTagPoses.addAll(tagPoses);
        allRobotPoses.addAll(robotPoses);
        allRobotPosesAccepted.addAll(robotPosesAccepted);
        allRobotPosesRejected.addAll(robotPosesRejected);
      }
    }

    // Log summary data
    if (logThisCycle) {
      Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
