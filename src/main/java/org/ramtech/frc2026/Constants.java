// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.ramtech.frc2026;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running on a roboRIO. Change the value of "simMode" to switch
 * between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
	public static final Mode simMode = Mode.SIM;
	public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
	public static final CANBus CANivore = new CANBus("RT Canivore");

	public static enum Mode {
		/** Running on a real robot. */
		REAL,

		/** Running a physics simulator. */
		SIM,

		/** Replaying from a log file. */
		REPLAY
	}

	public static final class IntakeConstants {
		public static final int intakePivotMotorId = 45;
		public static final int rollerMotorId = 47;
		public static final int intakeEncoderId = 0;
	}

	public static final class IndexerConstants {
		public static final int starMotorID = 50;
		public static final int ballTunnelMotorID = 51;
	}

	public static final class TowerConstants {
		public static final int towerMotorId = 53;
	}

	public static final class TurretConstants {
		public static final int turretMotorId = 54;
		public static final int turretEncoderAId = 55;
		public static final int turretEncoderBId = 56;
		public static final double rotorToSensorRatio = 1;
		public static final double SensorToMechanismRatio = 0.14668233692646027;

		public static final Double kP_Slot0 = 0.5;
		public static final Double kI_Slot0 = 0.004999999888241291;
		public static final Double kD_Slot0 = 0.0;
		public static final Double kS_Slot0 = 0.1796875;
		public static final Double kV_Slot0 = 0.0;
		public static final Double kA_Slot0 = 0.0;
		public static final Double kG_Slot0 = 0.0;
		public static final double motionMagicAcceleration = 9000;
		public static final double motionMagicCruiseVelocity = 770;
		public static final double motionMagicJerk = 0;

		public static final int StatorCurrentLimit = 120;
		public static final boolean StatorCurrentLimitEnable = true;
		public static final int SupplyCurrentLimit = 30;
		public static final boolean SupplyCurrentLimitEnable = true;
		public static final int SupplyCurrentLowerLimit = 30;
		public static final int SupplyCurrentLowerTime = 3;

		public static final double forwardSoftLimit = 600;
		public static final boolean forwardSoftLimitEnable = true;

		public static final double reverseSoftLimit = 60;
		public static final boolean reverseSoftLimitEnable = true;

	}

	public static final class HoodConstants {
		public static final int hoodMotorId = 57;
		public static final double rotorToSensorRatio = 1;
		public static final double SensorToMechanismRatio = 0.75;

		public static final Double kP_Slot0 = 3.5;
		public static final Double kI_Slot0 = 0.0002;
		public static final Double kD_Slot0 = 0.0;
		public static final Double kS_Slot0 = 0.03;
		public static final Double kV_Slot0 = 0.0;
		public static final Double kA_Slot0 = 0.00015;
		public static final Double kG_Slot0 = 0.0;

		public static final double motionMagicAcceleration = 9000;
		public static final double motionMagicCruiseVelocity = 770;
		public static final double motionMagicJerk = 0;

		public static final double forwardSoftLimit = 40;
		public static final boolean forwardSoftLimitEnable = true;

		public static final double reverseSoftLimit = 0;
		public static final boolean reverseSoftLimitEnable = true;
	}

	public static final class FlywheelConstants {
		public static final int leftMotorId = 58;
		public static final int rightMotorId = 59;

		public static final Double kP_Slot0 = 0.5;
		public static final Double kI_Slot0 = 2.0;
		public static final Double kD_Slot0 = 0.0;
		public static final Double kS_Slot0 = 0.0;
		public static final Double kV_Slot0 = 0.2;
		public static final Double kA_Slot0 = 0.0;
		public static final Double kG_Slot0 = 0.0;
		public static final Double peakForwardVoltage = 10.0;
		public static final Double peakReverseVoltage = 3.0;

	}

	public static final class TargetPoses {
		public static Pose3d hub = new Pose3d(4.65594, 4.034663, 1.8288, new Rotation3d());
		public static Pose3d leftFarPass = new Pose3d(4.125, 4.374663 + 1.5, 0.0, new Rotation3d());
		public static Pose3d rightFarPass = new Pose3d(4.125, 4.374663 - 1.5, 0.0, new Rotation3d());
		public static Pose3d leftClosePass = new Pose3d(4.125, 4.034663 + 1.5, 0.0, new Rotation3d());
		public static Pose3d rightClosePass = new Pose3d(4.125, 4.034663 - 1.5, 0.0, new Rotation3d());
	}

	public static final class Offsets {
		public static final Transform3d turretOffset = new Transform3d(Units.inchesToMeters(-9),
				Units.inchesToMeters(-3.25), Units.inchesToMeters(5), new Rotation3d());
		public static final Transform2d turretOffset2d = new Transform2d(turretOffset.getX(), turretOffset.getY(),
				new Rotation2d());
	}

	public static final class ClimbConstants {
		public static final int leftClimbMotorId = 60;
		public static final int rightClimbMotorId = 61;
	}
}
