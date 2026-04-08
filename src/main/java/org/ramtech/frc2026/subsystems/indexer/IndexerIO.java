package org.ramtech.frc2026.subsystems.indexer;

import com.ctre.phoenix6.StatusCode;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

	@AutoLog
	public static class IndexerIOInputs {
		public StatusCode signalsOk = StatusCode.NodeIsInvalid;

		public boolean indexerConnected = false;
		public boolean IndexerConfigured = false;

		public double IndexerMotorVoltage = 0.0;

		public double IndexerRps = 0.0;

		public double IndexerSupplyCurrent = 0.0;
	}

	public static enum IndexerIOOutputMode {
		OFF, VOLTAGE
	}

	public static enum IndexerIOAutoDirections {
		FORWARD, REVERSE
	}

	public static class IndexerIOOutputs {
		public IndexerIOOutputMode mode = IndexerIOOutputMode.OFF;
		public double indexerVoltageSetpoint = 0.0;
		public IndexerIOAutoDirections directionSetpoint = IndexerIOAutoDirections.FORWARD;
	}

	default void updateInputs(IndexerIOInputs inputs) {
	}

	default void applyOutputs(IndexerIOOutputs outputs) {
	}
}
