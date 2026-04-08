package org.ramtech.frc2026.subsystems.shooter.flywheel;

import static org.ramtech.frc2026.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.ramtech.frc2026.Constants;
import org.ramtech.frc2026.Constants.FlywheelConstants;

public class FlywheelIOTalonFX implements FlywheelIO {
	// Motors
	private final TalonFX leftFlywheelMotor = new TalonFX(FlywheelConstants.leftMotorId, Constants.CANivore);
	private final TalonFX rightFlywheelMotor = new TalonFX(FlywheelConstants.rightMotorId, Constants.CANivore);

	// Configuration
	private final TalonFXConfiguration leftSideConfig = new TalonFXConfiguration();
	private final TalonFXConfiguration rightSideConfig = new TalonFXConfiguration();
	private boolean leftSideConfigured = false;
	private boolean rightSideConfigured = false;

	private final StatusSignal<Voltage> leftMotorVoltageSig;
	private final StatusSignal<AngularVelocity> leftMotorVelocitySig;
	private final StatusSignal<Current> leftMotorCurrentSig;

	private final StatusSignal<Voltage> rightMotorVoltageSig;
	private final StatusSignal<AngularVelocity> rightMotorVelocitySig;
	private final StatusSignal<Current> rightMotorCurrentSig;

	// Control Methods
	private final VoltageOut voltageOut = new VoltageOut(0);
	private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

	private final StrictFollower follower = new StrictFollower(FlywheelConstants.leftMotorId);

	public FlywheelIOTalonFX() {
		// Complete the config
		// Left Side
		leftSideConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		leftSideConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		leftSideConfig.Slot0.kP = FlywheelConstants.kP_Slot0;
		leftSideConfig.Slot0.kI = FlywheelConstants.kI_Slot0;
		leftSideConfig.Slot0.kD = FlywheelConstants.kD_Slot0;
		leftSideConfig.Slot0.kS = FlywheelConstants.kS_Slot0;
		leftSideConfig.Slot0.kV = FlywheelConstants.kV_Slot0;
		leftSideConfig.Slot0.kA = FlywheelConstants.kA_Slot0;
		leftSideConfig.Slot0.kG = FlywheelConstants.kG_Slot0;
		leftSideConfig.Voltage.PeakForwardVoltage = FlywheelConstants.peakForwardVoltage;
		leftSideConfig.Voltage.PeakReverseVoltage = FlywheelConstants.peakReverseVoltage;
		leftSideConfig.CurrentLimits.StatorCurrentLimit = 20;
		leftSideConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		leftSideConfig.CurrentLimits.SupplyCurrentLimit = 20;
		leftSideConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		leftSideConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;
		leftSideConfig.CurrentLimits.SupplyCurrentLowerTime = 3;

		// Right Side
		rightSideConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		rightSideConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		rightSideConfig.CurrentLimits.StatorCurrentLimit = 20;
		rightSideConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		rightSideConfig.CurrentLimits.SupplyCurrentLimit = 20;
		rightSideConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		rightSideConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;
		rightSideConfig.CurrentLimits.SupplyCurrentLowerTime = 3;
		rightSideConfig.Voltage.PeakForwardVoltage = FlywheelConstants.peakForwardVoltage;
		rightSideConfig.Voltage.PeakReverseVoltage = FlywheelConstants.peakReverseVoltage;

		leftSideConfigured = tryUntilOkWithStatus(5, () -> leftFlywheelMotor.getConfigurator().apply(leftSideConfig));
		rightSideConfigured = tryUntilOkWithStatus(5,
				() -> rightFlywheelMotor.getConfigurator().apply(rightSideConfig));

		leftMotorVoltageSig = leftFlywheelMotor.getMotorVoltage();
		leftMotorVelocitySig = leftFlywheelMotor.getVelocity();
		leftMotorCurrentSig = leftFlywheelMotor.getSupplyCurrent();

		rightMotorVoltageSig = rightFlywheelMotor.getMotorVoltage();
		rightMotorVelocitySig = rightFlywheelMotor.getVelocity();
		rightMotorCurrentSig = rightFlywheelMotor.getSupplyCurrent();

		BaseStatusSignal.setUpdateFrequencyForAll(50.0, leftMotorVoltageSig, leftMotorVelocitySig, leftMotorCurrentSig,
				rightMotorVoltageSig, rightMotorVelocitySig, rightMotorCurrentSig);

		leftFlywheelMotor.optimizeBusUtilization();
		rightFlywheelMotor.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		inputs.leftSignalsOk = BaseStatusSignal.refreshAll(leftMotorVoltageSig, leftMotorVelocitySig,
				leftMotorCurrentSig);
		inputs.rightSignalsOk = BaseStatusSignal.refreshAll(rightMotorVoltageSig, rightMotorVelocitySig,
				rightMotorCurrentSig);

		inputs.leftSideConnected = BaseStatusSignal.isAllGood(leftMotorVoltageSig);
		inputs.rightSideConnected = BaseStatusSignal.isAllGood(rightMotorVoltageSig);

		inputs.leftSideConfigured = leftSideConfigured;
		inputs.rightSideConfigured = rightSideConfigured;

		inputs.leftSideMotorVoltage = leftMotorVoltageSig.getValueAsDouble();
		inputs.rightSideMotorVoltage = rightMotorVoltageSig.getValueAsDouble();

		inputs.leftSideVelocity = leftMotorVelocitySig.getValueAsDouble();
		inputs.rightSideVelocity = rightMotorVelocitySig.getValueAsDouble();

		inputs.leftSideSupplyCurrent = leftMotorCurrentSig.getValueAsDouble();
		inputs.rightSideSupplyCurrent = rightMotorCurrentSig.getValueAsDouble();
	}

	@Override
	public void applyOutputs(FlywheelIOOutputs outputs) {
		switch (outputs.mode) {
			case OFF :
				leftFlywheelMotor.stopMotor();
				rightFlywheelMotor.stopMotor();
				break;
			case VOLTAGE :
				leftFlywheelMotor.setControl(voltageOut.withOutput(outputs.voltageSetpoint).withEnableFOC(true));
				rightFlywheelMotor.setControl(follower);
				break;
			case VELOCITY :
				leftFlywheelMotor.setControl(velocityVoltage.withVelocity(outputs.velocitySetpoint).withEnableFOC(true)
						.withFeedForward(outputs.feedForward));
				rightFlywheelMotor.setControl(follower);
				break;
		}
	}
}
