package org.ramtech.frc2026.subsystems.intake;

import static org.ramtech.frc2026.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
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
import org.ramtech.frc2026.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
	// Motors
	private final TalonFX rollerMotor = new TalonFX(IntakeConstants.rollerMotorId, CANBus.roboRIO()); // Main Motor

	// Configuration
	private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
	private boolean rollerConfigured = false;

	// Status Signals (Cached to prevent allocation in loop)
	private final StatusSignal<Voltage> rollerVoltageSig;
	private final StatusSignal<AngularVelocity> rollerVelocitySig;
	private final StatusSignal<Current> rollerCurrentSig;

	// Control Methods
	private final VoltageOut voltageOut = new VoltageOut(0); // Control Method
	private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

	public IntakeIOTalonFX() {
		// Complete the config
		rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		rollerConfig.CurrentLimits.StatorCurrentLimit = 100;
		rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		rollerConfig.CurrentLimits.SupplyCurrentLimit = 50;
		rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		rollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
		rollerConfig.CurrentLimits.SupplyCurrentLowerTime = 3;

		// Configure Motors
		rollerConfigured = tryUntilOkWithStatus(5, () -> rollerMotor.getConfigurator().apply(rollerConfig, 0.25));

		// Initialize signals
		rollerVoltageSig = rollerMotor.getMotorVoltage();
		rollerVelocitySig = rollerMotor.getVelocity();
		rollerCurrentSig = rollerMotor.getSupplyCurrent();

		BaseStatusSignal.setUpdateFrequencyForAll(50.0, rollerVoltageSig, rollerVelocitySig, rollerCurrentSig);

		rollerMotor.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		inputs.signalsOk = BaseStatusSignal.refreshAll(rollerVoltageSig, rollerVelocitySig, rollerCurrentSig);

		// Configuration
		inputs.rollerConnected = BaseStatusSignal.isAllGood(rollerVoltageSig);
		inputs.rollerConfigured = rollerConfigured;
		inputs.rollerVoltage = rollerVoltageSig.getValueAsDouble();
		inputs.rollerRps = rollerVelocitySig.getValueAsDouble();
		inputs.rollerSupplyCurrent = rollerCurrentSig.getValueAsDouble();
	}

	@Override
	public void applyOutputs(IntakeIOOutputs outputs) {
		switch (outputs.mode) {
			case OFF :
				rollerMotor.stopMotor();
				break;
			case VOLTAGE :
				rollerMotor.setControl(voltageOut.withOutput(outputs.voltageSetpoint).withEnableFOC(true));
				break;
			case VELOCITY :
				rollerMotor.setControl(velocityVoltage.withVelocity(outputs.velocitySetpoint).withEnableFOC(true));
		}
	}
}
