package org.ramtech.frc2026.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.ramtech.frc2026.Constants;
import org.ramtech.frc2026.Constants.HoodConstants;

public class HoodIOTalonFX implements HoodIO {
  // Motors
  private final TalonFX hoodMotor = new TalonFX(HoodConstants.hoodMotorId, Constants.Canivore); // Main Motor

  // Configuration
  private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
  private boolean hoodConfigured = false;

  // Control Methods
  private final VoltageOut voltageOut = new VoltageOut(0); // Control Method
  private final PositionVoltage positionVoltage = new PositionVoltage(0);

  public HoodIOTalonFX() {
    // Complete the config
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Configuration
    inputs.hoodConnected = hoodMotor.isConnected(); // Detect connected
    if (!hoodConfigured && inputs.hoodConnected) { // Configure motor
      hoodMotor.getConfigurator().apply(hoodConfig);
      hoodConfigured = true;
    }

    inputs.hoodConfigured = hoodConfigured;

    inputs.hoodAppliedVoltage = hoodMotor.getMotorVoltage().getValueAsDouble();

    inputs.hoodPosition = hoodMotor.getPosition().getValueAsDouble();

    inputs.hoodSupplyCurrentAmps = hoodMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case OFF:
        hoodMotor.stopMotor();
        break;
      case VOLTAGE:
        hoodMotor.setControl(voltageOut.withOutput(outputs.voltageSetpoint));
        break;
      case POSITION:
        hoodMotor.setControl(positionVoltage.withVelocity(outputs.positionSetpoint));
    }
  }
}
