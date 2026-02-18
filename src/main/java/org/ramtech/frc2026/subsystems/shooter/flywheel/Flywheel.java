package org.ramtech.frc2026.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import org.littletonrobotics.junction.Logger;
import org.ramtech.frc2026.Robot;
import org.ramtech.frc2026.subsystems.shooter.ShotCalculator;
import org.ramtech.frc2026.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputMode;
import org.ramtech.frc2026.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import org.ramtech.frc2026.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOSetpointSource;
import org.ramtech.frc2026.util.ShooterSubsystem;

public class Flywheel extends ShooterSubsystem {
  private final Object outputsLock = new Object();

  // IO
  private final FlywheelIO io; // the different values and possiblities relating to the subsystem.
  private final FlywheelIOInputsAutoLogged inputs =
      new FlywheelIOInputsAutoLogged(); // the truth: what the motor sees
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs(); // the targets
  // Alerts
  private final Debouncer leftSideDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rightSideDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final Alert leftSideDisconnected =
      new Alert("Left Flywheel Motor Disconnected", Alert.AlertType.kWarning);
  private final Alert rightSideDisconnected =
      new Alert("Right Flywheel Motor Disconnected", Alert.AlertType.kWarning);

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs); // Grab new values from motor
    Logger.processInputs("Shooter/Flywheel", inputs); // Put values in the log
    // Alerts
    leftSideDisconnected.set(
        Robot.showHardwareAlerts() && !leftSideDebouncer.calculate(inputs.leftSideConnected));

    rightSideDisconnected.set(
        Robot.showHardwareAlerts() && !rightSideDebouncer.calculate(inputs.rightSideConnected));
  }

  @Override
  public void periodicAfterScheduler() {
    synchronized (outputsLock) {
      Logger.recordOutput("Shooter/Flywheel/Mode", outputs.mode);
      Logger.recordOutput("Shooter/Flywheel/VoltageSetpoint", outputs.voltageSetpoint);
      Logger.recordOutput("Shooter/Flywheel/VelocitySetpoint", outputs.velocitySetpoint);
    }
  }

  @Override
  public void shooterMotorPeriodic(double dt) {
    var shotCalculation = ShotCalculator.getInstance().getLatest();
    synchronized (outputsLock) {
      if (outputs.setpointSource == FlywheelIOSetpointSource.SHOT_CALCULATOR) {
        if (shotCalculation.isValid()) {
          outputs.mode = FlywheelIOOutputMode.VELOCITY;
          outputs.velocitySetpoint = shotCalculation.flywheelVelocity();
        }
      }
      io.applyOutputs(outputs); // Set the targets for the motor
    }
  }

  public void enableCalculation() {
    synchronized (outputsLock) {
      outputs.setpointSource = FlywheelIOSetpointSource.SHOT_CALCULATOR;
    }
  }

  public void disableCalculation() {
    synchronized (outputsLock) {
      outputs.setpointSource = FlywheelIOSetpointSource.MANUAL;
    }
  }

  public void setVoltage(double voltage) {
    synchronized (outputsLock) {
      outputs.setpointSource = FlywheelIOSetpointSource.MANUAL;
      outputs.mode = FlywheelIOOutputMode.VOLTAGE;
      outputs.voltageSetpoint = voltage;
    }
  }

  public void setVelocity(double velocity) {
    synchronized (outputsLock) {
      outputs.setpointSource = FlywheelIOSetpointSource.MANUAL;
      outputs.mode = FlywheelIOOutputMode.VELOCITY;
      outputs.velocitySetpoint = velocity;
    }
  }

  public void stop() {
    synchronized (outputsLock) {
      outputs.setpointSource = FlywheelIOSetpointSource.MANUAL;
      outputs.mode = FlywheelIOOutputMode.OFF;
      outputs.voltageSetpoint = 0.0;
      outputs.velocitySetpoint = 0.0;
    }
  }
}
