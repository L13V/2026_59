package org.ramtech.frc2026.subsystems.shooter.tower;

import static org.ramtech.frc2026.util.PhoenixUtil.tryUntilOkWithStatus;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
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
	private final TalonFX towerMotorA = new TalonFX(TowerConstants.towerMotorAId, Constants.CANivore); // Main Motor
	private final TalonFX towerMotorB = new TalonFX(TowerConstants.towerMotorBId, Constants.CANivore); // Main Motor

	// Configuration
	private final TalonFXConfiguration towerConfig = new TalonFXConfiguration();
	private boolean towerMotorAConfigured = false;
	private boolean towerMotorBConfigured = false;

	private final StatusSignal<Voltage> towerMotorAVoltageSig;
	private final StatusSignal<AngularVelocity> towerMotorAVelocitySig;
	private final StatusSignal<Current> towerMotorACurrentSig;

	private final StatusSignal<Voltage> towerMotorBVoltageSig;
	private final StatusSignal<AngularVelocity> towerMotorBVelocitySig;
	private final StatusSignal<Current> towerMotorBCurrentSig;

	// Control Methods
	private final VoltageOut voltageOut = new VoltageOut(0); // Control Method
	private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
	private final StrictFollower follower = new StrictFollower(TowerConstants.towerMotorAId);

	public TowerIOTalonFX() {
		// Complete the config
		towerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		towerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		towerConfig.CurrentLimits.StatorCurrentLimit = 120;
		towerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		towerConfig.CurrentLimits.SupplyCurrentLimit = 60;
		towerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		towerConfig.CurrentLimits.SupplyCurrentLowerLimit = 60;
		towerConfig.CurrentLimits.SupplyCurrentLowerTime = 1;

		towerMotorAConfigured = tryUntilOkWithStatus(5, () -> towerMotorA.getConfigurator().apply(towerConfig));
		towerMotorBConfigured = tryUntilOkWithStatus(5, () -> towerMotorB.getConfigurator().apply(towerConfig));

		// Initialize signals
		towerMotorAVoltageSig = towerMotorA.getMotorVoltage();
		towerMotorAVelocitySig = towerMotorA.getVelocity();
		towerMotorACurrentSig = towerMotorA.getSupplyCurrent();

		towerMotorBVoltageSig = towerMotorB.getMotorVoltage();
		towerMotorBVelocitySig = towerMotorB.getVelocity();
		towerMotorBCurrentSig = towerMotorB.getSupplyCurrent();
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, towerMotorAVoltageSig, towerMotorAVelocitySig,
				towerMotorACurrentSig, towerMotorBVoltageSig, towerMotorBVelocitySig, towerMotorBCurrentSig);

		towerMotorA.optimizeBusUtilization();
		towerMotorB.optimizeBusUtilization();

	}

	@Override
	public void updateInputs(TowerIOInputs inputs) {

		inputs.signalsOk = BaseStatusSignal.refreshAll(towerMotorAVoltageSig, towerMotorAVelocitySig,
				towerMotorACurrentSig, towerMotorBVoltageSig, towerMotorBVelocitySig, towerMotorBCurrentSig);

		inputs.towerMotorAConnected = BaseStatusSignal.isAllGood(towerMotorAVoltageSig);
		inputs.towerMotorBConnected = BaseStatusSignal.isAllGood(towerMotorBVoltageSig);

		inputs.towerMotorAConfigured = towerMotorAConfigured;
		inputs.towerMotorBConfigured = towerMotorBConfigured;

		inputs.towerMotorAVoltage = towerMotorAVoltageSig.getValueAsDouble();
		inputs.towerMotorAVelocity = towerMotorAVelocitySig.getValueAsDouble();
		inputs.towerMotorASupplyCurrent = towerMotorACurrentSig.getValueAsDouble();

		inputs.towerMotorBVoltage = towerMotorBVoltageSig.getValueAsDouble();
		inputs.towerMotorBVelocity = towerMotorBVelocitySig.getValueAsDouble();
		inputs.towerMotorBSupplyCurrent = towerMotorBCurrentSig.getValueAsDouble();
	}

	@Override
	public void applyOutputs(TowerIOOutputs outputs) {
		switch (outputs.mode) {
			case OFF :
				towerMotorA.stopMotor();
				towerMotorB.stopMotor();

				break;
			case VOLTAGE :
				towerMotorA.setControl(voltageOut.withOutput(outputs.voltageSetpoint).withEnableFOC(true));
				towerMotorB.setControl(follower);
				break;
			case VELOCITY :
				towerMotorA.setControl(velocityVoltage.withVelocity(outputs.velocitySetpoint).withEnableFOC(true));
				towerMotorB.setControl(follower);
				break;
		}

	}
}
