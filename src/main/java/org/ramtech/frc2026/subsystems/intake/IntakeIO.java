package org.ramtech.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusCode;

public interface IntakeIO {

	@AutoLog
	public static class IntakeIOInputs {
		public StatusCode signalsOk = StatusCode.NodeIsInvalid;

		public boolean motorAConnected = false;
		public boolean motorAConfigured = false;

		public boolean motorBConnected = false;
		public boolean motorBConfigured = false;

		public boolean intakePivotMotorConnected = false;
		public boolean intakePivotMotorConfigured = false;

		public double motorAVoltage = 0.0;
		public double motorARps = 0.0;
		public double motorASupplyCurrent = 0.0;

		public double motorBVoltage = 0.0;
		public double motorBRps = 0.0;
		public double motorBSupplyCurrent = 0.0;

		public double intakePivotMotorVoltage = 0.0;
		public double intakePivotMotorRps = 0.0;
		public double intakePivotMotorSupplyCurrent = 0.0;
		public double intakePivotMotorPosition = 0.0;

	}

	public static enum IntakeIORollerOutputMode {
		OFF, VOLTAGE, AUTO
	}

	public static enum IntakeIOPivotOutputMode {
		OFF, POSITION, LOWER
	}

	public static enum IntakeIOAutoDirections {
		FORWARD, REVERSE
	}

	@AutoLog
	public static class IntakeIOOutputs {
		public IntakeIORollerOutputMode rollerMode = IntakeIORollerOutputMode.OFF;
		public IntakeIOPivotOutputMode pivotMode = IntakeIOPivotOutputMode.OFF;
		public double rollerVoltageSetpoint = 0.0;
		public double pivotPositionSetpoint = 0.0;

		public IntakeIOAutoDirections directionSetpoint = IntakeIOAutoDirections.FORWARD;

	}

	default void updateInputs(IntakeIOInputs inputs) {
	}

	default void applyOutputs(IntakeIOOutputs outputs) {
	}
}
