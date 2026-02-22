// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.ramtech.frc2026.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Arrays;
import java.util.Queue;
import org.ramtech.frc2026.generated.TunerConstants;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
      pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
    } else {
      pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps = drainDoubleQueue(yawTimestampQueue);
    inputs.odometryYawPositions = drainDegreesQueue(yawPositionQueue);
  }

  private static double[] drainDoubleQueue(Queue<Double> queue) {
    int expectedSize = queue.size();
    double[] values = new double[expectedSize];
    int count = 0;
    for (; count < expectedSize; count++) {
      Double value = queue.poll();
      if (value == null) {
        break;
      }
      values[count] = value;
    }
    return count == expectedSize ? values : Arrays.copyOf(values, count);
  }

  private static Rotation2d[] drainDegreesQueue(Queue<Double> queue) {
    int expectedSize = queue.size();
    Rotation2d[] values = new Rotation2d[expectedSize];
    int count = 0;
    for (; count < expectedSize; count++) {
      Double value = queue.poll();
      if (value == null) {
        break;
      }
      values[count] = Rotation2d.fromDegrees(value);
    }
    return count == expectedSize ? values : Arrays.copyOf(values, count);
  }
}
