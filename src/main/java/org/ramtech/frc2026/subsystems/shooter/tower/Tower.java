// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ramtech.frc2026.subsystems.shooter.tower;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.ramtech.frc2026.Robot;
import org.ramtech.frc2026.subsystems.shooter.tower.TowerIO.TowerIOOutputMode;
import org.ramtech.frc2026.subsystems.shooter.tower.TowerIO.TowerIOOutputs;

public class Tower extends SubsystemBase {
  // IO
  private final TowerIO io;
  private final TowerIOInputsAutoLogged inputs = new TowerIOInputsAutoLogged();
  private final TowerIOOutputs outputs = new TowerIOOutputs();
  // Alerts
  private final Debouncer towerDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert towerDisconnected =
      new Alert("Tower Motor Disconnected!", Alert.AlertType.kWarning);

  /** Creates a new Tower. */
  public Tower(TowerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Tower", inputs);
    towerDisconnected.set(
        Robot.showHardwareAlerts() && !towerDebouncer.calculate(inputs.towerConnected));
    periodicAfterScheduler();
  }

  public void periodicAfterScheduler() {
    io.applyOutputs(outputs); // Set the targets for the motor
    Logger.recordOutput("Tower/Mode", outputs.mode);
    Logger.recordOutput("Tower/Voltage", outputs.voltage);
  }

  public void setVoltage(double voltage) {
    outputs.mode = TowerIOOutputMode.VOLTAGE;
    outputs.voltage = voltage;
  }

  public void stop() {
    outputs.mode = TowerIOOutputMode.COAST;
    outputs.voltage = 0.00;
  }
}
