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
import edu.wpi.first.units.measure.LinearAcceleration;
import java.util.Queue;
import org.ramtech.frc2026.generated.TunerConstants;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
	private final Pigeon2 pigeon = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
	private final StatusSignal<Angle> yaw = pigeon.getYaw();
	private final Queue<Double> yawPositionQueue;
	private final Queue<Double> yawTimestampQueue;
	private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZDevice();
	private final StatusSignal<LinearAcceleration> accelX = pigeon.getAccelerationX();
	private final StatusSignal<LinearAcceleration> accelY = pigeon.getAccelerationY();
	private final StatusSignal<LinearAcceleration> accelZ = pigeon.getAccelerationZ();

	public GyroIOPigeon2() {
		if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
			pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
		} else {
			pigeon.getConfigurator().apply(new Pigeon2Configuration());
		}

		pigeon.getConfigurator().setYaw(0.0);
		yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
		yawVelocity.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, accelX, accelY, accelZ);
		pigeon.optimizeBusUtilization();
		yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
		yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity, accelX, accelY, accelZ).equals(StatusCode.OK);
		inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
		inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
		inputs.accelX = accelX.getValueAsDouble();
		inputs.accelY = accelY.getValueAsDouble();
		inputs.accelZ = accelZ.getValueAsDouble();

		int yawTimestampCount = yawTimestampQueue.size();
		inputs.odometryYawTimestamps = new double[yawTimestampCount];
		for (int i = 0; i < yawTimestampCount; i++) {
			Double value = yawTimestampQueue.poll();
			inputs.odometryYawTimestamps[i] = value != null ? value : 0.0;
		}

		int yawPositionCount = yawPositionQueue.size();
		inputs.odometryYawPositions = new Rotation2d[yawPositionCount];
		for (int i = 0; i < yawPositionCount; i++) {
			Double value = yawPositionQueue.poll();
			inputs.odometryYawPositions[i] = value != null ? Rotation2d.fromDegrees(value) : Rotation2d.kZero;
		}
	}
}
