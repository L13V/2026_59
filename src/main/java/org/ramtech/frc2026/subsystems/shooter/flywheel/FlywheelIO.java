package org.ramtech.frc2026.subsystems.shooter.flywheel;

import com.ctre.phoenix6.StatusCode;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

	@AutoLog
	public static class FlywheelIOInputs {
		public StatusCode leftSignalsOk = StatusCode.NodeIsInvalid;
		public StatusCode rightSignalsOk = StatusCode.NodeIsInvalid;

		public boolean leftSideConnected = false;
		public boolean leftSideConfigured = false;

		public boolean rightSideConnected = false;
		public boolean rightSideConfigured = false;

		public double leftSideMotorVoltage = 0.0;
		public double rightSideMotorVoltage = 0.0;

		public double leftSideVelocity = 0.0;
		public double rightSideVelocity = 0.0;

		public double leftSideSupplyCurrent = 0.0;
		public double rightSideSupplyCurrent = 0.0;

	}

	public static enum FlywheelIOSetpointSource {
		MANUAL, SHOT_CALCULATOR
	}

	public static enum FlywheelIOOutputMode {
		OFF, VOLTAGE, VELOCITY
	}

	@AutoLog
	public static class FlywheelIOOutputs {
		public FlywheelIOOutputMode mode = FlywheelIOOutputMode.VELOCITY;
		public FlywheelIOSetpointSource setpointSource = FlywheelIOSetpointSource.SHOT_CALCULATOR;
		public double voltageSetpoint = 0.0;
		public double velocitySetpoint = 0.0;
		public double feedForward = 0.0;
	}

	default void updateInputs(FlywheelIOInputs inputs) {
	}

	default void applyOutputs(FlywheelIOOutputs outputs) {
	}
}
