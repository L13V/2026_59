// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.ramtech.frc2026;

import java.util.List;

import org.ramtech.frc2026.util.Zones.Zone;
import org.ramtech.frc2026.util.Zones.zoneType;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
	public static final boolean isComp = true;

	public static final Mode simMode = Mode.SIM;
	public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
	public static final CANBus CANivore = new CANBus("RT Canivore");
	public static final double MAX_WHEEL_ROTATION_VELOCITY = 10.0;

	public static boolean disableFlipping = false;

	public static enum Mode {
		/** Running on a real robot. */
		REAL,

		/** Running a physics simulator. */
		SIM,

		/** Replaying from a log file. */
		REPLAY
	}

	public static final class FieldConstants {
		public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
				.loadField(AprilTagFields.k2026RebuiltWelded);
		public static final double fieldLength = aprilTagLayout.getFieldLength();
		public static final double fieldWidth = aprilTagLayout.getFieldWidth();

		public static final Double ourAllianceHoodLower = 4.645;

		public static final Zone ourLeftTrench = new Zone(4.228694, 6.80339, 4.028694 + 1.1938 - 0.3, 8.069326,
				"Our Left Trench", zoneType.hoodUnsafe);
		public static final Zone ourRightTrench = new Zone(4.228694, 0, 4.028694 + 1.1938 - 0.3, 1.265936,
				"Our Right Trench", zoneType.hoodUnsafe);
		public static final Zone opposingLeftTrench = new Zone(11.318496, 0, 11.318496 + 1.1938, 1.265936,
				"Opposing Left Trench", zoneType.hoodUnsafe);
		public static final Zone opposingRightTrench = new Zone(11.318496, 6.80339, 11.318496 + 1.1938, 8.069326,
				"Opposing Right Trench", zoneType.hoodUnsafe);
		public static final Zone ourAlliance = new Zone(0, 0, 5.222494, 8.069326, "Our Alliance", zoneType.scoring);

		public static final Zone centerPassLeft = new Zone(5.222494, (8.069326 / 2) + 0.1, 11.318496, 8.069326,
				"Left Center", zoneType.passingcloseleft);
		public static final Zone centerPassRight = new Zone(5.222494, 0, 11.318496, (8.069326 / 2) - 0.1,
				"Right Center", zoneType.passingcloseright);

		public static final Zone farPassLeft = new Zone(11.318496, (8.069326 / 2) + 0.1, 16.540988, 8.069326,
				"Opposing Alliance Left", zoneType.passingfarleft);

		public static final Zone farPassRight = new Zone(11.318496, 0, 16.540988, (8.069326 / 2) - 0.1,
				"Opposing Alliance Right", zoneType.passingfarright);

		public static final List<Zone> allZones = List.of(ourLeftTrench, ourRightTrench, opposingLeftTrench,
				opposingRightTrench, ourAlliance, centerPassLeft, centerPassRight, farPassLeft, farPassRight);
	}

	public static final class TargetPoses {
		public static Pose3d hub = new Pose3d(4.645, 4.02, 1.82, new Rotation3d());
		public static Pose3d leftFarPass = new Pose3d(1.5, 4.02 + 2.3, 0.20, new Rotation3d());
		public static Pose3d rightFarPass = new Pose3d(1.5, 4.02 - 2.3, 0.20, new Rotation3d());
		public static Pose3d leftClosePass = new Pose3d(1.5, 4.02 + 2.3, 0.20, new Rotation3d());
		public static Pose3d rightClosePass = new Pose3d(1.5, 4.02 - 2.3, 0.20, new Rotation3d());

	}

	public static final class Offsets {
		public static final Transform3d turretOffset = new Transform3d(Units.inchesToMeters(-3.25),
				Units.inchesToMeters(-9), Units.inchesToMeters(5), new Rotation3d());
		public static final Transform2d turretOffset2d = new Transform2d(turretOffset.getX(), turretOffset.getY(),
				new Rotation2d());
	}

	public static final class IntakeConstants {
		public static final int pivotMotorID = 45;
		public static final int motorAID = 47;
		public static final int motorBID = 48;
		public static final int intakeEncoderId = 0;

		public static final double rotorToSensorRatio = 1;
		public static final double SensorToMechanismRatio = 50.8;

		public static final double kP_Slot0 = 30;
		public static final double kI_Slot0 = 0.2;
		public static final double kD_Slot0 = 0.0;
		public static final double kS_Slot0 = 0.0;
		public static final double kV_Slot0 = 0.0;
		public static final double kA_Slot0 = 0.0;
		public static final double kG_Slot0 = 0.5;
		public static final double motionMagicAcceleration = 150;
		public static final double motionMagicCruiseVelocity = 35;
		public static final double motionMagicJerk = 0;

		public static final double forwardSoftLimit = 0.29;
		public static final boolean forwardSoftLimitEnable = true;

		public static final double reverseSoftLimit = -0.1;
		public static final boolean reverseSoftLimitEnable = true;
	}

	public static final class IndexerConstants {
		public static final int indexerMotorID = 50;
	}

	public static final class TowerConstants {
		public static final int towerMotorAId = 52;

		public static final int towerMotorBId = 53;

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
		public static final Double kP_Slot1 = 0.5 / 5;
		public static final Double kI_Slot1 = 0.004999999888241291 / 5;
		public static final Double kD_Slot1 = 0.0 / 5;
		public static final Double kS_Slot1 = 0.1796875 / 5;
		public static final Double kV_Slot1 = 0.0;
		public static final Double kA_Slot1 = 0.0;
		public static final Double kG_Slot1 = 0.0;
		public static final double motionMagicAcceleration = 9000;
		public static final double motionMagicCruiseVelocity = 770;
		public static final double motionMagicJerk = 0;

		public static final int StatorCurrentLimit = 120;
		public static final boolean StatorCurrentLimitEnable = true;
		public static final int SupplyCurrentLimit = 30;
		public static final boolean SupplyCurrentLimitEnable = true;
		public static final int SupplyCurrentLowerLimit = 30;
		public static final int SupplyCurrentLowerTime = 3;

		public static final double turretAngleOffsetForZero = 0;

		public static final double forwardSoftLimit = 1015;
		public static final boolean forwardSoftLimitEnable = true;

		public static final double reverseSoftLimit = 200;
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

		public static final double forwardSoftLimit = 41;
		public static final boolean forwardSoftLimitEnable = true;

		public static final double reverseSoftLimit = 0;
		public static final boolean reverseSoftLimitEnable = true;
	}

	public static final class FlywheelConstants {
		public static final int leftMotorId = 58;
		public static final int rightMotorId = 59;

		public static final Double kP_Slot0 = 0.7;
		public static final Double kI_Slot0 = 2.0;
		public static final Double kD_Slot0 = 0.0;
		public static final Double kS_Slot0 = 0.0;
		public static final Double kV_Slot0 = 0.13;
		public static final Double kA_Slot0 = 0.0;
		public static final Double kG_Slot0 = 0.0;
		public static final Double peakForwardVoltage = 13.0;
		public static final Double peakReverseVoltage = 4.25;

	}

	public static final class ClimbConstants {
		public static final int leftClimbMotorId = 60;
		public static final int rightClimbMotorId = 61;
	}

	public static final class LedConstants {
		public static final int CANdleID = 60;
	}

}
