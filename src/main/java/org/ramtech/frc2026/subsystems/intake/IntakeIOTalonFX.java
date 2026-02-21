package org.ramtech.frc2026.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.ramtech.frc2026.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
  // Motors
  // private final TalonFX wheelMotor = new
  // TalonFX(IntakeConstants.intakeWheelsMotorId, Constants.CANivore); // Main
  // Motor
  private final TalonFX rollerMotor =
      new TalonFX(IntakeConstants.rollerMotorId); // Main Motor

  // Configuration
  private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
  private boolean rollerConfigured = false;

  // Control Methods
  private final VoltageOut rollerVoltageOut = new VoltageOut(0); // Control Method
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  public IntakeIOTalonFX() {
    // Complete the config
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // intakeWheelsConfig.CurrentLimits.StatorCurrentLimit = 120;
    // intakeWheelsConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // intakeWheelsConfig.CurrentLimits.SupplyCurrentLimit = 120;
    // intakeWheelsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    // intakeWheelsConfig.CurrentLimits.SupplyCurrentLowerLimit = 70;
    // intakeWheelsConfig.CurrentLimits.SupplyCurrentLowerTime = 3;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Configuration
    inputs.rollerConnected = rollerMotor.isConnected(); // Detect connected
    if (!rollerConfigured && inputs.rollerConnected) { // Configure motor
      rollerMotor.getConfigurator().apply(rollerConfig);
      rollerConfigured = true;
    }

    inputs.rollerConfigured = rollerConfigured;

    inputs.rollerVoltage = rollerMotor.getMotorVoltage().getValueAsDouble();

    inputs.rollerVelocity = rollerMotor.getVelocity().getValueAsDouble();

    inputs.rollerSupplyCurrent = rollerMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    switch (outputs.mode) {
      case OFF:
        rollerMotor.stopMotor();
        break;
      case VOLTAGE:
        rollerMotor.setControl(
            rollerVoltageOut.withOutput(outputs.voltageSetpoint).withEnableFOC(true));
        break;
      case VELOCITY:
        rollerMotor.setControl(velocityVoltage.withVelocity(outputs.velocitySetpoint));
    }
  }
}
