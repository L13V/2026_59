package org.ramtech.frc2026.subsystems.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.ramtech.frc2026.Robot;
import org.ramtech.frc2026.subsystems.indexer.IndexerIO.IndexerIOOutputMode;
import org.ramtech.frc2026.subsystems.indexer.IndexerIO.IndexerIOOutputs;
import org.ramtech.frc2026.util.FullSubsystem;

public class Indexer extends FullSubsystem {
  // IO
  private final IndexerIO io; // the different values and possiblities relating to the subsystem.
  private final IndexerIOInputsAutoLogged inputs =
      new IndexerIOInputsAutoLogged(); // the truth: what the motor sees
  private final IndexerIOOutputs outputs = new IndexerIOOutputs(); // the targets
  // Alerts
  private final Debouncer turretSideDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer intakeSideDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final Alert turretSideDisconnected =
      new Alert("Turret Indexer Motor Disconnected", Alert.AlertType.kWarning);
  private final Alert intakeSideDisconnected =
      new Alert("Intake Indexer Motor Disconnected", Alert.AlertType.kWarning);

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs); // Grab new values from motor
    Logger.processInputs("Indexer", inputs); // Put values in the log
    // Alerts
    turretSideDisconnected.set(
        Robot.showHardwareAlerts() && !turretSideDebouncer.calculate(inputs.turretSideConnected));

    intakeSideDisconnected.set(
        Robot.showHardwareAlerts() && !intakeSideDebouncer.calculate(inputs.intakeSideConnected));

    periodicAfterScheduler();
  }

  public void periodicAfterScheduler() {
    io.applyOutputs(outputs); // Set the targets for the motor
    Logger.recordOutput("Indexer/Mode", outputs.mode);
    Logger.recordOutput("Indexer/Voltage", outputs.voltage);
  }

  public void setVoltage(double voltage) {
    outputs.mode = IndexerIOOutputMode.VOLTAGE;
    outputs.voltage = voltage;
  }

  public void stop() {
    outputs.mode = IndexerIOOutputMode.COAST;
    outputs.voltage = 0.0;
  }
}
