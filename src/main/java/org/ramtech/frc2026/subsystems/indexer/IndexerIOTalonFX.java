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
	private final TalonFX indexerMotor = new TalonFX(IndexerConstants.indexerMotorID, Constants.CANivore);

	// Configuration
	private final TalonFXConfiguration indexer = new TalonFXConfiguration();
	private boolean indexerConfigured = false;

	// Status Signals (Cached to prevent allocation in loop)
	private final StatusSignal<Voltage> indexerVoltageSig;
	private final StatusSignal<AngularVelocity> indexerVelocitySig;
	private final StatusSignal<Current> indexerCurrentSig;

	// Control Methods
	private final VoltageOut ballTunnelVoltageOut = new VoltageOut(0);

	public IndexerIOTalonFX() {
		// Build Configs
		// Roller
		indexer.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		indexer.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		indexer.CurrentLimits.StatorCurrentLimit = 120;
		indexer.CurrentLimits.StatorCurrentLimitEnable = true;
		indexer.CurrentLimits.SupplyCurrentLimit = 30;
		indexer.CurrentLimits.SupplyCurrentLimitEnable = true;
		indexer.CurrentLimits.SupplyCurrentLowerLimit = 30;
		indexer.CurrentLimits.SupplyCurrentLowerTime = 3;

		// Configure Motors
		indexerConfigured = tryUntilOkWithStatus(5, () -> indexerMotor.getConfigurator().apply(indexer, 0.25));

		// Initialize signals
		indexerVoltageSig = indexerMotor.getMotorVoltage();
		indexerVelocitySig = indexerMotor.getVelocity();
		indexerCurrentSig = indexerMotor.getSupplyCurrent();

		BaseStatusSignal.setUpdateFrequencyForAll(50.0, indexerVoltageSig, indexerVelocitySig, indexerCurrentSig);

		indexerMotor.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(IndexerIOInputs inputs) {

		// Refresh all signals at once (efficient)
		inputs.signalsOk = BaseStatusSignal.refreshAll(indexerVoltageSig, indexerVelocitySig, indexerCurrentSig);

		// Ball Tunnel
		inputs.indexerConnected = BaseStatusSignal.isAllGood(indexerVoltageSig); // True if connected
		inputs.IndexerConfigured = indexerConfigured;
		inputs.IndexerMotorVoltage = indexerVoltageSig.getValueAsDouble();
		inputs.IndexerRps = indexerVelocitySig.getValueAsDouble();
		inputs.IndexerSupplyCurrent = indexerCurrentSig.getValueAsDouble();
	}

	@Override
	public void applyOutputs(IndexerIOOutputs outputs) {
		switch (outputs.mode) {
			case OFF :
				indexerMotor.stopMotor();
				break;

			case VOLTAGE :
				indexerMotor.setControl(
						ballTunnelVoltageOut.withOutput(outputs.indexerVoltageSetpoint).withEnableFOC(true));
				break;
		}
	}
}
