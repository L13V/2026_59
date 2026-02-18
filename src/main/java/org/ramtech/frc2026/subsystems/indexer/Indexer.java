package org.ramtech.frc2026.subsystems.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
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
  private final Debouncer spindexerDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer starDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final Alert spindexerDisconnected =
      new Alert("Turret Indexer Motor Disconnected", Alert.AlertType.kWarning);
  private final Alert starsDisconnected =
      new Alert("Star Motor Disconnected", Alert.AlertType.kWarning);

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs); // Grab new values from motor
    Logger.processInputs("Indexer", inputs); // Put values in the log
    // Alerts
    spindexerDisconnected.set(
        Robot.showHardwareAlerts() && !spindexerDebouncer.calculate(inputs.spindexerConnected));

    starsDisconnected.set(
        Robot.showHardwareAlerts() && !starDebouncer.calculate(inputs.starsConnected));
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs); // Set the targets for the motor
    Logger.recordOutput("Indexer/Mode", outputs.mode);
    Logger.recordOutput("Indexer/Spindexer Voltage", outputs.spindexerVoltage);
    Logger.recordOutput("Indexer/Star Voltage", outputs.starVoltage);
  }

  public void setVoltages(double spinVoltage, double starVoltage) {
    outputs.mode = IndexerIOOutputMode.VOLTAGE;
    outputs.spindexerVoltage = spinVoltage;
    outputs.starVoltage = starVoltage;
  }

  public void stop() {
    outputs.mode = IndexerIOOutputMode.COAST;
    outputs.spindexerVoltage = 0.0;
    outputs.starVoltage = 0.0;
  }
}
