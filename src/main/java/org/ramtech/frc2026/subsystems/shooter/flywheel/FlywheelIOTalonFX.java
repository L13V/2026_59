package org.ramtech.frc2026.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import org.ramtech.frc2026.Constants;
import org.ramtech.frc2026.Constants.FlywheelConstants;

public class FlywheelIOTalonFX implements FlywheelIO {
  // Motors
  private final TalonFX leftFlywheelMotor =
      new TalonFX(FlywheelConstants.leftMotorId, Constants.CANBus);
  private final TalonFX rightFlywheelMotor =
      new TalonFX(FlywheelConstants.rightMotorId, Constants.CANBus);

  // Configuration
  private final TalonFXConfiguration leftSideConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration rightSideConfig = new TalonFXConfiguration();
  private boolean leftSideConfigured = false;
  private boolean rightSideConfigured = false;

  // Control Methods
  private final VoltageOut voltageOut = new VoltageOut(0);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  private final StrictFollower follower = new StrictFollower(FlywheelConstants.leftMotorId);

  public FlywheelIOTalonFX() {
    // Complete the config
    // Left Side
    leftSideConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftSideConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leftSideConfig.Slot0.kP = FlywheelConstants.kP_Slot0;
    leftSideConfig.Slot0.kI = FlywheelConstants.kI_Slot0;
    leftSideConfig.Slot0.kD = FlywheelConstants.kD_Slot0;
    leftSideConfig.Slot0.kS = FlywheelConstants.kS_Slot0;
    leftSideConfig.Slot0.kV = FlywheelConstants.kV_Slot0;
    leftSideConfig.Slot0.kA = FlywheelConstants.kA_Slot0;
    leftSideConfig.Slot0.kG = FlywheelConstants.kG_Slot0;
    leftSideConfig.Voltage.PeakForwardVoltage = FlywheelConstants.peakForwardVoltage;
    leftSideConfig.Voltage.PeakReverseVoltage = FlywheelConstants.peakReverseVoltage;
    // Right Side
    rightSideConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightSideConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Configuration
    inputs.leftSideConnected = leftFlywheelMotor.isConnected();
    inputs.rightSideConnected = rightFlywheelMotor.isConnected();

    if (!leftSideConfigured && inputs.leftSideConnected) {
      leftFlywheelMotor.getConfigurator().apply(leftSideConfig);
      leftSideConfigured = true;
    }
    if (!rightSideConfigured && inputs.rightSideConnected) {
      rightFlywheelMotor.getConfigurator().apply(rightSideConfig);
      rightFlywheelMotor.setControl(follower);
      rightSideConfigured = true;
    }

    inputs.leftSideConfigured = leftSideConfigured;
    inputs.rightSideConfigured = rightSideConfigured;

    inputs.leftSideAppliedVoltage = leftFlywheelMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightSideAppliedVoltage = rightFlywheelMotor.getMotorVoltage().getValueAsDouble();

    inputs.leftSideVelocity = leftFlywheelMotor.getVelocity().getValueAsDouble();
    inputs.rightSideVelocity = rightFlywheelMotor.getVelocity().getValueAsDouble();

    inputs.leftSideSupplyCurrentAmps = leftFlywheelMotor.getSupplyCurrent().getValueAsDouble();
    inputs.rightSideSupplyCurrentAmps = rightFlywheelMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    switch (outputs.mode) {
      case OFF:
        leftFlywheelMotor.stopMotor();
        rightFlywheelMotor.stopMotor();
        break;
      case VOLTAGE:
        leftFlywheelMotor.setControl(voltageOut.withOutput(outputs.voltageSetpoint));
        rightFlywheelMotor.setControl(follower);
        break;
      case VELOCITY:
        leftFlywheelMotor.setControl(velocityVoltage.withVelocity(outputs.velocitySetpoint));
        rightFlywheelMotor.setControl(follower);
        break;
    }
  }
}
