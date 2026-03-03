package org.ramtech.frc2026.subsystems.shooter.turret;

import com.ctre.phoenix6.StatusCode;

import yams.units.EasyCRT.CRTStatus;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

	@AutoLog
	public static class TurretIOInputs {
		public StatusCode turretMotorSignalOk = StatusCode.NodeIsInvalid;
		public StatusCode turretEncoderASignalOk = StatusCode.NodeIsInvalid;
		public StatusCode turretEncoderBSignalOk = StatusCode.NodeIsInvalid;

		public boolean turretMotorConnected = false;
		public boolean turretMotorConfigured = false;

		public boolean turretEncoderAConnected = false;
		public boolean turretEncoderAConfigured = false;
		public boolean turretEncoderBConnected = false;
		public boolean turretEncoderBConfigured = false;

		public boolean turretCrtComplete = false;

		public double TurretMotorVoltage = 0.0;
		public double TurretMotorSupplyCurrent = 0.0;
		public double TurretMotorPosition = 0.0;
		public double turretMotorVelocity = 0.0;

		public double TurretEncoderAPosition = 0.0;
		public double TurretEncoderBPosition = 0.0;
		public double TurretEncoderAAbsPosition = 0.0;
		public double TurretEncoderBAbsPosition = 0.0;

		public double crtValue = 0.0;
		public CRTStatus crtStatus;
	}

	public static enum TurretIOSetpointSource {
		SHOT_CALCULATOR, MANUAL
	}

	public static enum TurretIOOutputMode {
		OFF, VOLTAGE, POSITION
	}

	@AutoLog
	public static class TurretIOOutputs {
		public TurretIOOutputMode mode = TurretIOOutputMode.POSITION;
		public TurretIOSetpointSource setpointSource = TurretIOSetpointSource.SHOT_CALCULATOR;
		public double voltageSetpoint = 0.0;
		public double positionSetpoint = 0.0;
	}

	default void updateInputs(TurretIOInputs inputs) {
	}

	default void applyOutputs(TurretIOOutputs outputs) {
	}
}
