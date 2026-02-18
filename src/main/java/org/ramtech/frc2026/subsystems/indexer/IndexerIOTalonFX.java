package org.ramtech.frc2026.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.*;
import org.ramtech.frc2026.Constants;
import org.ramtech.frc2026.Constants.IndexerConstants;

public class IndexerIOTalonFX implements IndexerIO {
  // Motors
  private final TalonFX spindexerMotor =
      new TalonFX(IndexerConstants.spindexerMotorID, Constants.Canivore);
  private final TalonFXS starMotor = new TalonFXS(IndexerConstants.starMotorID, Constants.Canivore);

  // Configuration
  private final TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
  private final TalonFXSConfiguration starConfig = new TalonFXSConfiguration();
  private boolean spindexerConfigured = false;
  private boolean starConfigured = false;

  // Control Methods
  private final VoltageOut spindexerVoltageOut = new VoltageOut(0);
  private final VoltageOut starVoltageOut = new VoltageOut(0);

  // private final StrictFollower follower = new
  // StrictFollower(IndexerConstants.indexerTurretSideMotorID);

  public IndexerIOTalonFX() {
    // Complete the config
    // Turret Side
    spindexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Intake Side
    starConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    starConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    starConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Configuration
    inputs.spindexerConnected = spindexerMotor.isConnected();
    inputs.starsConnected = starMotor.isConnected();

    if (!spindexerConfigured && inputs.spindexerConnected) {
      spindexerMotor.getConfigurator().apply(spindexerConfig);
      spindexerConfigured = true;
    }
    if (!starConfigured && inputs.starsConnected) {
      starMotor.getConfigurator().apply(starConfig);
      starConfigured = true;
    }

    inputs.spindexerConfigured = spindexerConfigured;
    inputs.starsConfigured = starConfigured;

    inputs.spindexerAppliedVoltage = spindexerMotor.getMotorVoltage().getValueAsDouble();
    inputs.starAppliedVoltage = starMotor.getMotorVoltage().getValueAsDouble();

    inputs.spindexerVelocity = spindexerMotor.getVelocity().getValueAsDouble();
    inputs.starVelocity = starMotor.getVelocity().getValueAsDouble();

    inputs.spindexerSupplyCurrentAmps = spindexerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.starSupplyCurrentAmps = starMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST:
        spindexerMotor.stopMotor();
        starMotor.stopMotor();
        break;

      case VOLTAGE:
        spindexerMotor.setControl(spindexerVoltageOut.withOutput(outputs.spindexerVoltage));
        starMotor.setControl(starVoltageOut.withOutput(outputs.starVoltage));
        break;
    }
  }
}
