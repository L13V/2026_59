package org.ramtech.frc2026.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  public static class IndexerIOInputs {
    public boolean spindexerConnected = false;
    public boolean spindexerConfigured = false;

    public boolean starsConnected = false;
    public boolean starsConfigured = false;

    public double spindexerAppliedVoltage = 0.0;
    public double starAppliedVoltage = 0.0;

    public double spindexerVelocity = 0.0;
    public double starVelocity = 0.0;

    public double spindexerSupplyCurrentAmps = 0.0;
    public double starSupplyCurrentAmps = 0.0;
  }

  public static enum IndexerIOOutputMode {
    COAST,
    VOLTAGE
  }

  public static class IndexerIOOutputs {
    public IndexerIOOutputMode mode = IndexerIOOutputMode.COAST;
    public double spindexerVoltage = 0.0;
    public double starVoltage = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void applyOutputs(IndexerIOOutputs outputs) {}
}
