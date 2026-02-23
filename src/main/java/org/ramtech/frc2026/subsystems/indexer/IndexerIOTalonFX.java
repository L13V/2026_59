package org.ramtech.frc2026.subsystems.indexer;

import static org.ramtech.frc2026.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.ramtech.frc2026.Constants;
import org.ramtech.frc2026.Constants.IndexerConstants;

public class IndexerIOTalonFX implements IndexerIO {
  // Motors
  private final TalonFX ballTunnelMotor =
      new TalonFX(IndexerConstants.ballTunnelMotorID, Constants.CANivore);
  private final TalonFX starMotor = new TalonFX(IndexerConstants.starMotorID, Constants.CANivore);

  // Configuration
  private final TalonFXConfiguration ballTunnelConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration starConfig = new TalonFXConfiguration();
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

  public IndexerIOTalonFX() {
    // Build Configs
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
    starConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    starConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Configure Motors
    ballTunnelConfigured =
        tryUntilOkWithStatus(
            5, () -> ballTunnelMotor.getConfigurator().apply(ballTunnelConfig, 0.25));
    starsConfigured =
        tryUntilOkWithStatus(5, () -> starMotor.getConfigurator().apply(starConfig, 0.25));

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

    ballTunnelMotor.optimizeBusUtilization();
    starMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {

    // Refresh all signals at once (efficient)
    inputs.signalsOk =
        BaseStatusSignal.refreshAll(
            ballTunnelVoltageSig,
            ballTunnelVelocitySig,
            ballTunnelCurrentSig,
            starVoltageSig,
            starVelocitySig,
            starCurrentSig);

    // Ball Tunnel
    inputs.ballTunnelConnected =
        BaseStatusSignal.isAllGood(ballTunnelVoltageSig); // True if connected
    inputs.ballTunnelConfigured = ballTunnelConfigured;
    inputs.ballTunnelMotorVoltage = ballTunnelVoltageSig.getValueAsDouble();
    inputs.ballTunnelVelocity = ballTunnelVelocitySig.getValueAsDouble();
    inputs.ballTunnelSupplyCurrent = ballTunnelCurrentSig.getValueAsDouble();
    // Stars
    inputs.starsConnected = BaseStatusSignal.isAllGood(starVoltageSig); // True if connected
    inputs.starsConfigured = starsConfigured;
    inputs.starMotorVoltage = starVoltageSig.getValueAsDouble();
    inputs.starVelocity = starVelocitySig.getValueAsDouble();
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
