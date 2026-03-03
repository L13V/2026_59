// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.ramtech.frc2026;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
		public static final Double kP_Slot0 = 0.0;
		public static final Double kI_Slot0 = 0.0;
		public static final Double kD_Slot0 = 0.0;
		public static final Double kS_Slot0 = 0.0;
		public static final Double kV_Slot0 = 0.0;
		public static final Double kA_Slot0 = 0.0;
		public static final Double kG_Slot0 = 0.0;
		public static final double motionMagicAcceleration = 0;
		public static final double motionMagicCruiseVelocity = 0;
		public static final double motionMagicJerk = 0;
		public static final double rotationLowerLimit = 60;
		public static final double rotationUpperLimit = 600; // TODO: Fix this

		public static final int StatorCurrentLimit = 120;
		public static final boolean StatorCurrentLimitEnable = true;
		public static final int SupplyCurrentLimit = 120;
		public static final boolean SupplyCurrentLimitEnable = true;
		public static final int SupplyCurrentLowerLimit = 70;
		public static final int SupplyCurrentLowerTime = 3;

	}

	public static final class HoodConstants {
		public static final int hoodMotorId = 57;
		public static final Double kP_Slot0 = 0.0;
		public static final Double kI_Slot0 = 0.0;
		public static final Double kD_Slot0 = 0.0;
		public static final Double kS_Slot0 = 0.0;
		public static final Double kV_Slot0 = 0.0;
		public static final Double kA_Slot0 = 0.0;
		public static final Double kG_Slot0 = 0.0;
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
		public static final Double peakForwardVoltage = 16.0;
		public static final Double peakReverseVoltage = 3.0;
	}

	public static final class TargetPoses {
		public static Pose3d hub = new Pose3d(4.611624, 4.021328, 1.0, new Rotation3d());
		public static Pose3d leftPass = new Pose3d(1.0, 1.0, 1.0, new Rotation3d());
		public static Pose3d rightPass = new Pose3d(1.0, 1.0, 1.0, new Rotation3d());
	}

	public static final class Offsets {
		public static final Transform3d turretOffset = new Transform3d(Units.inchesToMeters(9),
				Units.inchesToMeters(-3.25), Units.inchesToMeters(5), new Rotation3d());
	}

	public static final class ClimbConstants {
		public static final int leftClimbMotorId = 60;
		public static final int rightClimbMotorId = 61;
	}
}
