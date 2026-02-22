// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.ramtech.frc2026.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final AHRS navX = new AHRS(NavXComType.kMXP_SPI, (byte) Drive.ODOMETRY_FREQUENCY);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getYaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(-navX.getYaw());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());

    int queueSize = yawTimestampQueue.size();
    double[] timestamps = new double[queueSize];
    Rotation2d[] yaws = new Rotation2d[queueSize];
    for (int i = 0; i < queueSize; i++) {
      Double timestamp = yawTimestampQueue.poll();
      Double yaw = yawPositionQueue.poll();
      if (timestamp == null || yaw == null) {
        // Should not happen if queues are synchronized
        break;
      }
      timestamps[i] = timestamp;
      yaws[i] = Rotation2d.fromDegrees(-yaw);
    }
    inputs.odometryYawTimestamps = timestamps;
    inputs.odometryYawPositions = yaws;
  }
}
