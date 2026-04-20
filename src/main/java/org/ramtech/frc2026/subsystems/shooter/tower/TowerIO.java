package org.ramtech.frc2026.subsystems.shooter.tower;

import com.ctre.phoenix6.StatusCode;
import org.littletonrobotics.junction.AutoLog;

public interface TowerIO {

	@AutoLog
	public static class TowerIOInputs {
		public StatusCode signalsOk = StatusCode.NodeIsInvalid;

		public boolean towerMotorAConnected = false;
		public boolean towerMotorAConfigured = false;

		public double towerMotorAVoltage = 0.0;
		public double towerMotorAVelocity = 0.0;
		public double towerMotorASupplyCurrent = 0.0;

		public boolean towerMotorBConnected = false;
		public boolean towerMotorBConfigured = false;

		public double towerMotorBVoltage = 0.0;
		public double towerMotorBVelocity = 0.0;
		public double towerMotorBSupplyCurrent = 0.0;
	}

	public static enum TowerIOOutputMode {
		OFF, VOLTAGE, VELOCITY
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
