package org.ramtech.frc2026.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.*;
import org.ramtech.frc2026.Constants;
import org.ramtech.frc2026.Constants.IndexerConstants;

public class IndexerIOTalonFXS implements IndexerIO {
  // Motors
  private final TalonFXS turretSideMotor =
      new TalonFXS(IndexerConstants.indexerTurretSideMotorID, Constants.CANBus);
  private final TalonFXS intakeSideMotor =
      new TalonFXS(IndexerConstants.indexerIntakeSideMotorID, Constants.CANBus);

  // Configuration
  TalonFXSConfiguration turretSideConfig = new TalonFXSConfiguration();
  TalonFXSConfiguration intakeSideConfig = new TalonFXSConfiguration();
  private boolean turretSideConfgured = false;
  private boolean intakeSideConfgured = false;

  // Control Methods
  private final VoltageOut voltageOut = new VoltageOut(0);
  private final StrictFollower follower =
      new StrictFollower(IndexerConstants.indexerTurretSideMotorID);

  public IndexerIOTalonFXS() {
    // Complete the config
    // Turret Side
    turretSideConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turretSideConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    turretSideConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
    turretSideConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    // Intake Side
    intakeSideConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeSideConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Configuration
    inputs.turretSideConnected = turretSideMotor.isConnected();
    inputs.intakeSideConnected = intakeSideMotor.isConnected();

    if (!turretSideConfgured && inputs.turretSideConnected) {
      turretSideMotor.getConfigurator().apply(turretSideConfig);
      turretSideConfgured = true;
    }
    if (!intakeSideConfgured && inputs.intakeSideConnected) {
      intakeSideMotor.getConfigurator().apply(intakeSideConfig);
      intakeSideMotor.setControl(follower);
      intakeSideConfgured = true;
    }

    inputs.turretSideAppliedVoltage = turretSideMotor.getMotorVoltage().getValueAsDouble();
    inputs.intakeSideAppliedVoltage = intakeSideMotor.getMotorVoltage().getValueAsDouble();

    inputs.turretSideSupplyCurrentAmps = turretSideMotor.getSupplyCurrent().getValueAsDouble();
    inputs.intakeSideSupplyCurrentAmps = intakeSideMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST:
        turretSideMotor.stopMotor();
        intakeSideMotor.stopMotor();
        break;

      case VOLTAGE:
        turretSideMotor.setControl(voltageOut.withOutput(outputs.voltage));
        intakeSideMotor.setControl(follower);
        break;
    }
  }
}
