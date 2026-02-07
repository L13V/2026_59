// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Robot;

public class Flywheel extends SubsystemBase {
  // Motors
  static TalonFX leftFlywheelMotor = new TalonFX(FlywheelConstants.leftMotorId);
  static TalonFX rightFlywheelMotor = new TalonFX(FlywheelConstants.rightMotorId);
  // Configurations
  static TalonFXConfiguration leftFlywheelConfig = new TalonFXConfiguration();
  static TalonFXConfiguration rightFlywheelConfig = new TalonFXConfiguration();
  private boolean leftConfigured = false;
  private boolean rightConfigured = false;
  // Controls
  static VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  // Alerts
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer motorFollowerConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert leftConnectionAlert;
  private final Alert rightConnectionAlert;
  private boolean leftConnected = false;
  private boolean rightConnected = false;

  public Flywheel() {
    // Alerts
    leftConnectionAlert = new Alert("Left Flywheel Motor Disconnected!", Alert.AlertType.kWarning);
    rightConnectionAlert = new Alert("Right Flywheel Motor Disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    leftConnected = leftFlywheelMotor.isConnected();
    rightConnected = rightFlywheelMotor.isConnected();
    // Alerts
    leftConnectionAlert.set(
        Robot.showHardwareAlerts()
            && !motorConnectedDebouncer.calculate(leftConnected));
    rightConnectionAlert.set(
        Robot.showHardwareAlerts()
            && !motorFollowerConnectedDebouncer.calculate(rightConnected));

    /*
     * Configuration
     */
    // TODO: Complete implementation of motor configs (put vars and stuff)
    if (!leftConfigured && leftConnected) {
      leftFlywheelMotor.getConfigurator().refresh(leftFlywheelConfig);

      leftFlywheelMotor.getConfigurator().apply(leftFlywheelConfig);
      leftConfigured = true;
    }
    if (!rightConfigured && rightConnected) {
      rightFlywheelMotor.getConfigurator().refresh(rightFlywheelConfig);

      rightFlywheelMotor.getConfigurator().apply(rightFlywheelConfig);
      rightConfigured = true;
    }
  }
}
