package org.ramtech.frc2026.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.ramtech.frc2026.Constants;
import org.ramtech.frc2026.Constants.IndexerConstants;

public class IndexerIOTalonFX implements IndexerIO {
  // Motors
  private final TalonFX ballTunnelMotor = new TalonFX(IndexerConstants.ballTunnelMotorID, Constants.CANivore);
  private final TalonFXS starMotor = new TalonFXS(IndexerConstants.starMotorID, Constants.CANivore);

  // Configuration
  private final TalonFXConfiguration ballTunnelConfig = new TalonFXConfiguration();
  private final TalonFXSConfiguration starConfig = new TalonFXSConfiguration();
  private boolean ballTunnelConfigured = false;
  private boolean starsConfigured = false;

  // Status Signals (Cached to prevent allocation in loop)
  private final StatusSignal<Voltage> ballTunnelVoltageSig;
  private final StatusSignal<AngularVelocity> ballTunnelVelocitySig;
  private final StatusSignal<Current> ballTunnelCurrentSig;
  private final StatusSignal<Voltage> starVoltageSig;
  private final StatusSignal<AngularVelocity> starVelocitySig;
  private final StatusSignal<Current> starCurrentSig;

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

    // Initialize signals
    ballTunnelVoltageSig = ballTunnelMotor.getMotorVoltage();
    ballTunnelVelocitySig = ballTunnelMotor.getVelocity();
    ballTunnelCurrentSig = ballTunnelMotor.getSupplyCurrent();

    starVoltageSig = starMotor.getMotorVoltage();
    starVelocitySig = starMotor.getVelocity();
    starCurrentSig = starMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        ballTunnelVoltageSig,
        ballTunnelVelocitySig,
        ballTunnelCurrentSig,
        starVoltageSig,
        starVelocitySig,
        starCurrentSig);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Configuration
    inputs.ballTunnelConnected = ballTunnelMotor.isConnected();
    inputs.starsConnected = starMotor.isConnected();

    // Refresh all signals at once (efficient)
    inputs.signalsOk = BaseStatusSignal.refreshAll(
        ballTunnelVoltageSig,
        ballTunnelVelocitySig,
        ballTunnelCurrentSig,
        starVoltageSig,
        starVelocitySig,
        starCurrentSig);

    if (!ballTunnelConfigured && inputs.ballTunnelConnected) {
      ballTunnelMotor.getConfigurator().apply(ballTunnelConfig);
      ballTunnelConfigured = true;
    }
    if (!starsConfigured && inputs.starsConnected) {
      starMotor.getConfigurator().apply(starConfig);
      starsConfigured = true;
    }
    inputs.ballTunnelConfigured = true;
    inputs.starsConfigured = true;

    inputs.ballTunnelConfigured = ballTunnelConfigured;
    inputs.starsConfigured = starsConfigured;

    inputs.ballTunnelMotorVoltage = ballTunnelVoltageSig.getValueAsDouble();
    inputs.starMotorVoltage = starVoltageSig.getValueAsDouble();
    inputs.ballTunnelVelocity = ballTunnelVelocitySig.getValueAsDouble();
    inputs.starVelocity = starVelocitySig.getValueAsDouble();
    inputs.ballTunnelSupplyCurrent = ballTunnelCurrentSig.getValueAsDouble();
    inputs.starSupplyCurrent = starCurrentSig.getValueAsDouble();

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
