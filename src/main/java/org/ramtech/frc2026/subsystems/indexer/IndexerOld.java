// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ramtech.frc2026.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ramtech.frc2026.Constants.IndexerConstants;
import org.ramtech.frc2026.Robot;

public class IndexerOld extends SubsystemBase {
  // Motors
  static TalonFXS turretSideMotor =
      new TalonFXS(IndexerConstants.indexerTurretSideMotorID, "RT Canivore");
  static TalonFXS intakeSideMotor =
      new TalonFXS(IndexerConstants.indexerIntakeSideMotorID, "RT Canivore");
  // Configurations
  static TalonFXSConfiguration turretSideConfig = new TalonFXSConfiguration();
  static TalonFXSConfiguration intakeSideConfig = new TalonFXSConfiguration();
  private boolean turretSideConfigured = false;
  private boolean intakeSideConfigured = false;
  // Controls
  static VoltageOut voltageOut = new VoltageOut(0);
  static StrictFollower strictFollower =
      new StrictFollower(IndexerConstants.indexerIntakeSideMotorID);

  // Alerts
  private final Debouncer turretSideMotorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer intakeSideMotorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert turretSideMotorConnectionAlert;
  private final Alert intakeSideMotorConnectionAlert;
  private boolean turretSideMotorConnected = false;
  private boolean intakeSideMotorConnected = false;

  /** Creates a new Indexer. */
  public IndexerOld() {
    // Alerts
    turretSideMotorConnectionAlert =
        new Alert("Turret Side Indexer Motor Disconnected!", AlertType.kWarning);
    intakeSideMotorConnectionAlert =
        new Alert("Intake Side Indexer Motor Disconnected!", AlertType.kWarning);
  }

  @Override
  public void periodic() {
    turretSideMotorConnected = turretSideMotor.isConnected();
    intakeSideMotorConnected = intakeSideMotor.isConnected();

    turretSideMotorConnectionAlert.set(
        Robot.showHardwareAlerts()
            && !turretSideMotorConnectedDebouncer.calculate(turretSideMotorConnected));
    intakeSideMotorConnectionAlert.set(
        Robot.showHardwareAlerts()
            && !intakeSideMotorConnectedDebouncer.calculate(intakeSideMotorConnected));

    if (DriverStation.isDisabled() && !turretSideConfigured && turretSideMotorConnected) {
      turretSideConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      turretSideConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      turretSideConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
      turretSideConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
      turretSideMotor.getConfigurator().apply(turretSideConfig);
      turretSideMotor.setControl(voltageOut.withOutput(0));
      turretSideConfigured = true;
    }
    if (DriverStation.isDisabled() && !intakeSideConfigured && intakeSideMotorConnected) {
      intakeSideConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      intakeSideConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      intakeSideMotor.getConfigurator().apply(intakeSideConfig);
      intakeSideMotor.setControl(strictFollower);
      intakeSideConfigured = true;
    }

    // This method will be called once per scheduler run
  }

  public void setVoltage(double Voltage) {
    // intakeSideMotor.setControl(voltageOut.withOutput(Voltage));
    // turretSideMotor.setControl(strictFollower);
    turretSideMotor.setControl(voltageOut.withOutput(Voltage));
    intakeSideMotor.setControl(strictFollower);
  }

  public void stop() {
    turretSideMotor.stopMotor();
    intakeSideMotor.stopMotor();
  }
}
