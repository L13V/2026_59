package org.ramtech.frc2026.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.ramtech.frc2026.Constants;
import org.ramtech.frc2026.Constants.HoodConstants;

public class HoodIOTalonFX implements HoodIO {
  // Motors
  private final TalonFXS hoodMotor =
      new TalonFXS(HoodConstants.hoodMotorId, Constants.CANivore); // Main Motor

  // Configuration
  private final TalonFXSConfiguration hoodConfig = new TalonFXSConfiguration();
  private boolean hoodConfigured = false;

  // Control Methods
  private final VoltageOut voltageOut = new VoltageOut(0); // Control Method
  private final PositionVoltage positionVoltage = new PositionVoltage(0);

  public HoodIOTalonFX() {
    // Complete the config
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
    hoodConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    hoodConfig.Slot0.kP = HoodConstants.kP_Slot0;
    hoodConfig.Slot0.kI = HoodConstants.kI_Slot0;
    hoodConfig.Slot0.kD = HoodConstants.kD_Slot0;
    hoodConfig.Slot0.kS = HoodConstants.kS_Slot0;
    hoodConfig.Slot0.kV = HoodConstants.kV_Slot0;
    hoodConfig.Slot0.kA = HoodConstants.kA_Slot0;
    hoodConfig.Slot0.kG = HoodConstants.kG_Slot0;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodConfig.CurrentLimits.StatorCurrentLimit = 120;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.SupplyCurrentLimit = 120;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.SupplyCurrentLowerLimit = 70;
    hoodConfig.CurrentLimits.SupplyCurrentLowerTime = 3;
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

    inputs.hoodMotorVoltage = hoodMotor.getMotorVoltage().getValueAsDouble();

    inputs.hoodPosition = hoodMotor.getPosition().getValueAsDouble();
    inputs.hoodVelocity = hoodMotor.getVelocity().getValueAsDouble();
    inputs.hoodSupplyCurrent = hoodMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case OFF:
        hoodMotor.stopMotor();
        break;
      case VOLTAGE:
        hoodMotor.setControl(voltageOut.withOutput(outputs.voltageSetpoint).withEnableFOC(true));
        break;
      case POSITION:
        hoodMotor.setControl(
            positionVoltage.withPosition(outputs.positionSetpoint).withEnableFOC(true));
        break;
    }
  }
}
