package org.ramtech.frc2026.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

	@AutoLog
	public static class IntakeIOInputs {
		public StatusCode signalsOk = StatusCode.NodeIsInvalid;

		public boolean rollerConnected = false;
		public boolean rollerConfigured = false;

		public double rollerVoltage = 0.0;
		public double rollerRps = 0.0;
		public double rollerSupplyCurrent = 0.0;
	}

	public static enum IntakeIOOutputMode {
		OFF, VOLTAGE, VELOCITY
	}

	@AutoLog
	public static class IntakeIOOutputs {
		public IntakeIOOutputMode mode = IntakeIOOutputMode.OFF;
		public double voltageSetpoint = 0.0;
		public double velocitySetpoint = 0.0;
	}

	default void updateInputs(IntakeIOInputs inputs) {
	}

	default void applyOutputs(IntakeIOOutputs outputs) {
	}
}
