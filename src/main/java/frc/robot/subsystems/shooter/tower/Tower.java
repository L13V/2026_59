// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.tower;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.TowerConstants;

public class Tower extends SubsystemBase {
  // Motors
  static TalonFX towerMotor = new TalonFX(TowerConstants.towerMotorId);
  // Configurations
  static TalonFXConfiguration towerConfig = new TalonFXConfiguration();
  private boolean towerConfigured = false;
  // Controls
  static VoltageOut voltageOut = new VoltageOut(0);

  // Alerts
  private final Debouncer towerMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert towerConnectionAlert;
  private boolean towerConnected = false;

  public Tower() {
    // Alerts
    towerConnectionAlert = new Alert("Tower Motor Disconnected!", Alert.AlertType.kWarning);

  }

  @Override
  public void periodic() {
    towerConnected = towerMotor.isConnected();
    // Alerts
    towerConnectionAlert.set(Robot.showHardwareAlerts() && !towerMotorConnectedDebouncer.calculate(towerConnected));

    /*
     * Configuration
     */
    // TODO: Complete implementation of motor configs (put vars and stuff)
    if (!towerConfigured && towerConnected) {
      towerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      towerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      towerMotor.getConfigurator().apply(towerConfig);
      towerMotor.setControl(voltageOut.withOutput(0));
      towerConfigured = true;
    }
  }// TODO make fishman happy

  public void setVoltage(double voltage) {
    towerMotor.setControl(voltageOut.withOutput(voltage));
  }

  public void stop() {
    towerMotor.stopMotor();
  }
}
