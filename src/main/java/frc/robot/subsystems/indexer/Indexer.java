// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  // Motors
  static TalonFX intakeSideMotor = new TalonFX(IndexerConstants.indexerIntakeSideMotorID);
  static TalonFX turretSideMotor = new TalonFX(IndexerConstants.indexerTurretSideMotorID);
  // Configurations
  static TalonFXConfiguration intakeSideConfig = new TalonFXConfiguration();
  static TalonFXConfiguration turretSideConfig = new TalonFXConfiguration();
  private boolean intakeSideConfigured = false;
  private boolean turretSideConfigured = false;
  // Controls
  static VoltageOut voltageOut = new VoltageOut(0);
  static StrictFollower strictFollower = new StrictFollower(IndexerConstants.indexerIntakeSideMotorID);

  // Alerts
  private final Debouncer intakeSideMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turretSideMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert intakeSideMotorConnectionAlert;
  private final Alert turretSideMotorConnectionAlert;
  private boolean intakeSideMotorConnected = false;
  private boolean turretSideMotorConnected = false;

  /** Creates a new Indexer. */
  public Indexer() {
    // Alerts
    intakeSideMotorConnectionAlert = new Alert("Indexer Side Indexer Motor Disconnected!", AlertType.kWarning);
    turretSideMotorConnectionAlert = new Alert("Turret Side Indexer Motor Disconnected!", AlertType.kWarning);

  }

  @Override
  public void periodic() {
    intakeSideMotorConnected = intakeSideMotor.isConnected();
    turretSideMotorConnected = turretSideMotor.isConnected();

    intakeSideMotorConnectionAlert
        .set(Robot.showHardwareAlerts() && !intakeSideMotorConnectedDebouncer.calculate(intakeSideMotorConnected));
    turretSideMotorConnectionAlert
        .set(Robot.showHardwareAlerts() && !turretSideMotorConnectedDebouncer.calculate(turretSideMotorConnected));

    if (!intakeSideConfigured && intakeSideMotorConnected) {
      intakeSideConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      intakeSideConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      intakeSideMotor.getConfigurator().apply(intakeSideConfig);
      intakeSideMotor.setControl(voltageOut.withOutput(0));
      intakeSideConfigured = true;
    }
    if (!turretSideConfigured && turretSideConfigured) {
      turretSideConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      turretSideConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      turretSideMotor.getConfigurator().apply(intakeSideConfig);
      turretSideMotor.setControl(strictFollower);
      intakeSideConfigured = true;
    }
    // This method will be called once per scheduler run
  }

  public void setVoltage(double Voltage) {
    intakeSideMotor.setControl(voltageOut.withOutput(Voltage));
    turretSideMotor.setControl(strictFollower);
  }

  public void stop() {
    intakeSideMotor.stopMotor();
    turretSideMotor.stopMotor();
  }
}
