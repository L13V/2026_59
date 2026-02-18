package org.ramtech.frc2026.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public boolean leftSideConnected = false;
    public boolean leftSideConfigured = false;

    public boolean rightSideConnected = false;
    public boolean rightSideConfigured = false;

    public double leftSideAppliedVoltage = 0.0;
    public double rightSideAppliedVoltage = 0.0;

    public double leftSideVelocity = 0.0;
    public double rightSideVelocity = 0.0;

    public double leftSideSupplyCurrentAmps = 0.0;
    public double rightSideSupplyCurrentAmps = 0.0;
  }

  public static enum FlywheelIOSetpointSource {
    SHOT_CALCULATOR,
    MANUAL
  }

  public static enum FlywheelIOOutputMode {
    OFF,
    VOLTAGE,
    VELOCITY
  }

  public static class FlywheelIOOutputs {
    public FlywheelIOOutputMode mode = FlywheelIOOutputMode.OFF;
    public FlywheelIOSetpointSource setpointSource = FlywheelIOSetpointSource.SHOT_CALCULATOR;
    public double voltageSetpoint = 0.0;
    public double velocitySetpoint = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void applyOutputs(FlywheelIOOutputs outputs) {}
}
