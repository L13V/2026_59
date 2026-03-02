package org.ramtech.frc2026.subsystems.indexer;

import com.ctre.phoenix6.StatusCode;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

	@AutoLog
	public static class IndexerIOInputs {
		public StatusCode signalsOk = StatusCode.NodeIsInvalid;

		public boolean ballTunnelConnected = false;
		public boolean ballTunnelConfigured = false;

		public boolean starsConnected = false;
		public boolean starsConfigured = false;

		public double ballTunnelMotorVoltage = 0.0;
		public double starMotorVoltage = 0.0;

		public double ballTunnelRps = 0.0;
		public double starRps = 0.0;

		public double ballTunnelSupplyCurrent = 0.0;
		public double starSupplyCurrent = 0.0;
	}

	public static enum IndexerIOOutputMode {
		OFF, VOLTAGE
	}

	public static class IndexerIOOutputs {
		public IndexerIOOutputMode mode = IndexerIOOutputMode.OFF;
		public double ballTunnelVoltageSetpoint = 0.0;
		public double starVoltageSetpoint = 0.0;
	}

	default void updateInputs(IndexerIOInputs inputs) {
	}

	default void applyOutputs(IndexerIOOutputs outputs) {
	}
}
