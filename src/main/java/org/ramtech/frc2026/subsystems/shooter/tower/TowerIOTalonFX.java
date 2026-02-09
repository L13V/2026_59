package org.ramtech.frc2026.subsystems.shooter.tower;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.ramtech.frc2026.Constants;
import org.ramtech.frc2026.Constants.TowerConstants;

public class TowerIOTalonFX implements TowerIO {
  // Motors
  private final TalonFX towerMotor =
      new TalonFX(TowerConstants.towerMotorId, Constants.CANBus); // Main Motor

  // Configuration
  TalonFXConfiguration towerConfig = new TalonFXConfiguration();
  private boolean towerConfigured = false;

  // Control Methods
  private final VoltageOut voltageOut = new VoltageOut(0); // Control Method

  public TowerIOTalonFX() {
    // Complete the config
    towerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    towerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  }

  @Override
  public void updateInputs(TowerIOInputs inputs) {
    // Configuration
    inputs.towerConnected = towerMotor.isConnected(); // Detect connected
    if (!towerConfigured && inputs.towerConnected) { // Configure motor
      towerMotor.getConfigurator().apply(towerConfig);
      towerConfigured = true;
    }

    inputs.towerAppliedVoltage = towerMotor.getMotorVoltage().getValueAsDouble();

    inputs.towerSupplyCurrentAmps = towerMotor.getSupplyCurrent().getValueAsDouble();
  }

  public void applyOutputs(TowerIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST:
        towerMotor.stopMotor();
        break;

      case VOLTAGE:
        towerMotor.setControl(voltageOut.withOutput(outputs.voltage));
        break;
    }
  }
}
