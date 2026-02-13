package org.ramtech.frc2026.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import org.littletonrobotics.junction.Logger;
import org.ramtech.frc2026.Robot;
import org.ramtech.frc2026.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputMode;
import org.ramtech.frc2026.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import org.ramtech.frc2026.util.FullSubsystem;

public class Flywheel extends FullSubsystem {
  // IO
  private final FlywheelIO io; // the different values and possiblities relating to the subsystem.
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged(); // the truth: what the motor sees
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs(); // the targets
  // Alerts
  private final Debouncer leftSideDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rightSideDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final Alert leftSideDisconnected = new Alert("Left Flywheel Motor Disconnected", Alert.AlertType.kWarning);
  private final Alert rightSideDisconnected = new Alert("Right Flywheel Motor Disconnected", Alert.AlertType.kWarning);

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
    io.applyOutputs(outputs); // Set the targets for the motor
    Logger.recordOutput("Shooter/Flywheel/Mode", outputs.mode);
    Logger.recordOutput("Shooter/Flywheel/VoltageSetpoint", outputs.voltageSetpoint);
    Logger.recordOutput("Shooter/Flywheel/VelocitySetpoint", outputs.velocitySetpoint);

  }

  public void setVoltage(double voltage) {
    outputs.mode = FlywheelIOOutputMode.VOLTAGE;
    outputs.voltageSetpoint = voltage;
  }

  public void setVelocity(double velocity) {
    outputs.mode = FlywheelIOOutputMode.VELOCITY;
    outputs.velocitySetpoint = velocity;
  }

  public void stop() {
    outputs.mode = FlywheelIOOutputMode.COAST;
    outputs.voltageSetpoint = 0.0;
    outputs.velocitySetpoint = 0.0;
  }
}
