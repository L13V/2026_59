package org.ramtech.frc2026.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusCode;

public interface HoodIO {

	@AutoLog
	public static class HoodIOInputs {
		public StatusCode signalsOk = StatusCode.NodeIsInvalid;

		public boolean hoodConnected = false;
		public boolean hoodConfigured = false;

		public double hoodMotorVoltage = 0.0;
		public double hoodSupplyCurrent = 0.0;
		public double hoodPosition = 0.0;
		public double hoodVelocity = 0.0;
	}

	public static enum HoodIOSetpointSource {
		SHOT_CALCULATOR, MANUAL
	}

	public static enum HoodIOOutputMode {
		OFF, VOLTAGE, POSITION
	}

	@AutoLog
	public static class HoodIOOutputs {
		public HoodIOOutputMode mode = HoodIOOutputMode.POSITION;
		public HoodIOSetpointSource setpointSource = HoodIOSetpointSource.SHOT_CALCULATOR;
		public double voltageSetpoint = 0.0;
		public double positionSetpoint = 0.0;
	}

	default void updateInputs(HoodIOInputs inputs) {
	}

	default void applyOutputs(HoodIOOutputs outputs) {
	}
}
