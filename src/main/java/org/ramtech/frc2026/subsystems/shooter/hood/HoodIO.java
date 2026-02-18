package org.ramtech.frc2026.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public boolean hoodConnected = false;
    public boolean hoodConfigured = false;

    public double hoodAppliedVoltage = 0.0;
    public double hoodPosition = 0.0;
    public double hoodSupplyCurrentAmps = 0.0;
  }

  public static enum HoodIOSetpointSource {
    MANUAL,
    SHOT_CALCULATOR
  }

  public static enum HoodIOOutputMode {
    OFF,
    VOLTAGE,
    POSITION
  }

  public static class HoodIOOutputs {
    public HoodIOSetpointSource setpointSource = HoodIOSetpointSource.SHOT_CALCULATOR;
    public HoodIOOutputMode mode = HoodIOOutputMode.OFF;
    public double voltageSetpoint = 0.0;
    public double positionSetpoint = 0.0;
  }

  default void updateInputs(HoodIOInputs inputs) {
  }

  default void applyOutputs(HoodIOOutputs outputs) {
  }
}
