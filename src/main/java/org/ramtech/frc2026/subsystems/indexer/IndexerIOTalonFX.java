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
  private final TalonFX ballTunnelMotor =
      new TalonFX(IndexerConstants.ballTunnelMotorID, Constants.CANivore);
  private final TalonFXS starMotor = new TalonFXS(IndexerConstants.starMotorID, Constants.CANivore);

  // Configuration
  private final TalonFXConfiguration ballTunnelConfig = new TalonFXConfiguration();
  private final TalonFXSConfiguration starConfig = new TalonFXSConfiguration();
  private boolean ballTunnelConfigured = false;
  private boolean starConfigured = false;

  // Control Methods
  private final VoltageOut ballTunnelVoltageOut = new VoltageOut(0);
  private final VoltageOut starVoltageOut = new VoltageOut(0);

  // private final StrictFollower follower = new
  // StrictFollower(IndexerConstants.indexerTurretSideMotorID);

  public IndexerIOTalonFX() {
    // Complete the config
    // Ball Tunnel
    ballTunnelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    ballTunnelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    ballTunnelConfig.CurrentLimits.StatorCurrentLimit = 120;
    ballTunnelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    ballTunnelConfig.CurrentLimits.SupplyCurrentLimit = 120;
    ballTunnelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    ballTunnelConfig.CurrentLimits.SupplyCurrentLowerLimit = 70;
    ballTunnelConfig.CurrentLimits.SupplyCurrentLowerTime = 3;

    // Intake Side
    starConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    starConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    starConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Configuration
    inputs.ballTunnelConnected = ballTunnelMotor.isConnected();
    inputs.starsConnected = starMotor.isConnected();

    if (!ballTunnelConfigured && inputs.ballTunnelConnected) {
      ballTunnelMotor.getConfigurator().apply(ballTunnelConfig);
      ballTunnelConfigured = true;
    }
    if (!starConfigured && inputs.starsConnected) {
      starMotor.getConfigurator().apply(starConfig);
      starConfigured = true;
    }

    inputs.ballTunnelConfigured = ballTunnelConfigured;
    inputs.starsConfigured = starConfigured;

    inputs.ballTunnelMotorVoltage = ballTunnelMotor.getMotorVoltage().getValueAsDouble();
    inputs.starMotorVoltage = starMotor.getMotorVoltage().getValueAsDouble();

    inputs.ballTunnelVelocity = ballTunnelMotor.getVelocity().getValueAsDouble();
    inputs.starVelocity = starMotor.getVelocity().getValueAsDouble();

    inputs.ballTunnelSupplyCurrent = ballTunnelMotor.getSupplyCurrent().getValueAsDouble();
    inputs.starSupplyCurrent = starMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {
    switch (outputs.mode) {
      case OFF:
        ballTunnelMotor.stopMotor();
        starMotor.stopMotor();
        break;

      case VOLTAGE:
        ballTunnelMotor.setControl(
            ballTunnelVoltageOut.withOutput(outputs.ballTunnelVoltageSetpoint).withEnableFOC(true));
        starMotor.setControl(starVoltageOut.withOutput(outputs.starVoltageSetpoint));
        break;
    }
  }
}
