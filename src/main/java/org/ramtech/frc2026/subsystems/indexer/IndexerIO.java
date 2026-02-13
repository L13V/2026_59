package org.ramtech.frc2026.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  public static class IndexerIOInputs {
    public boolean turretSideConnected = false;
    public boolean turretSideConfigured = false;

    public boolean intakeSideConnected = false;
    public boolean intakeSideConfigured = false;

    public double turretSideAppliedVoltage = 0.0;
    public double intakeSideAppliedVoltage = 0.0;

    public double turretSideSupplyCurrentAmps = 0.0;
    public double intakeSideSupplyCurrentAmps = 0.0;
  }

  public static enum IndexerIOOutputMode {
    COAST,
    VOLTAGE
  }

  public static class IndexerIOOutputs {
    public IndexerIOOutputMode mode = IndexerIOOutputMode.COAST;
    public double voltage = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void applyOutputs(IndexerIOOutputs outputs) {}
}
