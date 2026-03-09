package org.ramtech.frc2026.subsystems.shooter.tower;

import static org.ramtech.frc2026.util.PhoenixUtil.tryUntilOkWithStatus;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.ramtech.frc2026.Constants;
import org.ramtech.frc2026.Constants.TowerConstants;

public class TowerIOTalonFX implements TowerIO {
	// Motors
	private final TalonFX towerMotor = new TalonFX(TowerConstants.towerMotorId, Constants.CANivore); // Main Motor

	// Configuration
	private final TalonFXConfiguration towerConfig = new TalonFXConfiguration();
	private boolean towerConfigured = false;

	private final StatusSignal<Voltage> towerVoltageSig;
	private final StatusSignal<AngularVelocity> towerVelocitySig;
	private final StatusSignal<Current> towerCurrentSig;

	// Control Methods
	private final VoltageOut voltageOut = new VoltageOut(0); // Control Method
	private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

	public TowerIOTalonFX() {
		// Complete the config
		towerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		towerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		towerConfig.CurrentLimits.StatorCurrentLimit = 120;
		towerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		towerConfig.CurrentLimits.SupplyCurrentLimit = 30;
		towerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		towerConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
		towerConfig.CurrentLimits.SupplyCurrentLowerTime = 3;

		towerConfigured = tryUntilOkWithStatus(5, () -> towerMotor.getConfigurator().apply(towerConfig));

		// Initialize signals
		towerVoltageSig = towerMotor.getMotorVoltage();
		towerVelocitySig = towerMotor.getVelocity();
		towerCurrentSig = towerMotor.getSupplyCurrent();

		BaseStatusSignal.setUpdateFrequencyForAll(50.0, towerVoltageSig, towerVelocitySig, towerCurrentSig);

		towerMotor.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(TowerIOInputs inputs) {

		inputs.signalsOk = BaseStatusSignal.refreshAll(towerVoltageSig, towerVelocitySig, towerCurrentSig);

		inputs.towerConnected = BaseStatusSignal.isAllGood(towerVoltageSig);
		inputs.towerConfigured = towerConfigured;

		inputs.towerMotorVoltage = towerVoltageSig.getValueAsDouble();
		inputs.towerVelocity = towerVelocitySig.getValueAsDouble();
		inputs.towerSupplyCurrent = towerCurrentSig.getValueAsDouble();
	}

	@Override
	public void applyOutputs(TowerIOOutputs outputs) {
		switch (outputs.mode) {
			case OFF :
				towerMotor.stopMotor();
				break;
			case VOLTAGE :
				towerMotor.setControl(voltageOut.withOutput(outputs.voltageSetpoint).withEnableFOC(true));
				break;
			case VELOCITY :
				towerMotor.setControl(velocityVoltage.withVelocity(outputs.velocitySetpoint).withEnableFOC(true));
		}
	}
}
