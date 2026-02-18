package org.ramtech.frc2026.subsystems.shooter.tower;

import org.littletonrobotics.junction.AutoLog;

public interface TowerIO {

  @AutoLog
  public static class TowerIOInputs {
    public boolean towerConnected = false;
    public boolean towerConfigured = false;

    public double towerAppliedVoltage = 0.0;
    public double towerVelocity = 0.0;
    public double towerSupplyCurrentAmps = 0.0;
  }

  public static enum TowerIOSetpointSource {
    MANUAL,
    SHOT_CALCULATOR
  }

  public static enum TowerIOOutputMode {
    OFF,
    VOLTAGE,
    VELOCITY
  }

  public static class TowerIOOutputs {
    public TowerIOSetpointSource setpointSource = TowerIOSetpointSource.SHOT_CALCULATOR;
    public TowerIOOutputMode mode = TowerIOOutputMode.OFF;
    public double voltage = 0.0;
    public double velocity = 0.0;
  }

  default void updateInputs(TowerIOInputs inputs) {}

  default void applyOutputs(TowerIOOutputs outputs) {}
}
