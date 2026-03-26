package org.ramtech.frc2026.subsystems.shooter.tower;

import com.ctre.phoenix6.StatusCode;
import org.littletonrobotics.junction.AutoLog;

public interface TowerIO {

	@AutoLog
	public static class TowerIOInputs {
		public StatusCode signalsOk = StatusCode.NodeIsInvalid;

		public boolean towerConnected = false;
		public boolean towerConfigured = false;

		public double towerMotorVoltage = 0.0;
		public double towerVelocity = 0.0;
		public double towerSupplyCurrent = 0.0;
	}

	public static enum TowerIOOutputMode {
		OFF, VOLTAGE, VELOCITY, AUTO
	}

	public static enum TowerIOAutoDirections {
		FORWARD, REVERSE
	}

	@AutoLog
	public static class TowerIOOutputs {
		public TowerIOOutputMode mode = TowerIOOutputMode.OFF;
		public double voltageSetpoint = 0.0;
		public double velocitySetpoint = 0.0;
		public TowerIOAutoDirections directionSetpoint = TowerIOAutoDirections.FORWARD;
	}

	default void updateInputs(TowerIOInputs inputs) {
	}

	default void applyOutputs(TowerIOOutputs outputs) {
	}
}
