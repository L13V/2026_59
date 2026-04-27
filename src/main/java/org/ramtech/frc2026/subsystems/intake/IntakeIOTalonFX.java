package org.ramtech.frc2026.subsystems.intake;

import static org.ramtech.frc2026.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.ramtech.frc2026.Constants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import org.ramtech.frc2026.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
	// Motors
	private final TalonFX motorA = new TalonFX(IntakeConstants.motorAID, Constants.CANivore); // Main Motor
	private final TalonFX motorB = new TalonFX(IntakeConstants.motorBID, Constants.CANivore); // Secondary Motor

	private final TalonFX pivotMotor = new TalonFX(IntakeConstants.pivotMotorID, Constants.CANivore); // Secondary Motor

	// Configuration
	private final TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
	private final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();

	private boolean motorAConfigured = false;
	private boolean motorBConfigured = false;
	private boolean pivotMotorConfigured = false;

	// Status Signals (Cached to prevent allocation in loop)
	private final StatusSignal<Voltage> motorAVoltageSig;
	private final StatusSignal<AngularVelocity> motorAVelocitySig;
	private final StatusSignal<Current> motorACurrentSig;
	private final StatusSignal<Voltage> motorBVoltageSig;
	private final StatusSignal<AngularVelocity> motorBVelocitySig;
	private final StatusSignal<Current> motorBCurrentSig;

	private final StatusSignal<Voltage> intakePivotVoltageSig;
	private final StatusSignal<AngularVelocity> intakePivotVelocitySig;
	private final StatusSignal<Current> intakePivotCurrentSig;
	private final StatusSignal<Angle> intakePivotPositionSig;
	// Control Methods
	private final VoltageOut voltageOut = new VoltageOut(0); // Control Method
	private final StrictFollower follower = new StrictFollower(IntakeConstants.motorAID);
	// private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);

	private final MotionMagicVoltage pivotPosition = new MotionMagicVoltage(0.0);

	public IntakeIOTalonFX() {
		// Complete the config
		intakeMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 120;
		intakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = 80;
		intakeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		intakeMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
		intakeMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.5;

		pivotMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		pivotMotorConfig.CurrentLimits.StatorCurrentLimit = 120;
		pivotMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		pivotMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
		pivotMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		pivotMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
		pivotMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 3;

		pivotMotorConfig.Slot0.kP = IntakeConstants.kP_Slot0;
		pivotMotorConfig.Slot0.kI = IntakeConstants.kI_Slot0;
		pivotMotorConfig.Slot0.kD = IntakeConstants.kD_Slot0;
		pivotMotorConfig.Slot0.kS = IntakeConstants.kS_Slot0;
		pivotMotorConfig.Slot0.kV = IntakeConstants.kV_Slot0;
		pivotMotorConfig.Slot0.kA = IntakeConstants.kA_Slot0;
		pivotMotorConfig.Slot0.kG = IntakeConstants.kG_Slot0;
		pivotMotorConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.motionMagicAcceleration;
		pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.motionMagicCruiseVelocity;
		pivotMotorConfig.MotionMagic.MotionMagicJerk = IntakeConstants.motionMagicJerk;
		pivotMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

		pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = IntakeConstants.forwardSoftLimitEnable;
		pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.forwardSoftLimit;
		pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = IntakeConstants.reverseSoftLimitEnable;
		pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.reverseSoftLimit;

		pivotMotorConfig.Feedback.SensorToMechanismRatio = IntakeConstants.SensorToMechanismRatio;

		// Configure Motors
		motorAConfigured = tryUntilOkWithStatus(5, () -> motorA.getConfigurator().apply(intakeMotorConfig, 0.25));
		motorBConfigured = tryUntilOkWithStatus(5, () -> motorB.getConfigurator().apply(intakeMotorConfig, 0.25));
		pivotMotorConfigured = tryUntilOkWithStatus(5,
				() -> pivotMotor.getConfigurator().apply(pivotMotorConfig, 0.25));

		// Initialize signals
		motorAVoltageSig = motorA.getMotorVoltage();
		motorAVelocitySig = motorA.getVelocity();
		motorACurrentSig = motorA.getSupplyCurrent();
		motorBVoltageSig = motorB.getMotorVoltage();
		motorBVelocitySig = motorB.getVelocity();
		motorBCurrentSig = motorB.getSupplyCurrent();

		intakePivotVoltageSig = pivotMotor.getMotorVoltage();
		intakePivotVelocitySig = pivotMotor.getVelocity();
		intakePivotPositionSig = pivotMotor.getPosition();
		intakePivotCurrentSig = pivotMotor.getSupplyCurrent();

		BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorAVoltageSig, motorAVelocitySig, motorACurrentSig,
				motorBVoltageSig, motorBVelocitySig, motorBCurrentSig, intakePivotVoltageSig, intakePivotVelocitySig,
				intakePivotCurrentSig, intakePivotPositionSig);

		motorA.optimizeBusUtilization();
		motorB.optimizeBusUtilization();
		pivotMotor.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		inputs.signalsOk = BaseStatusSignal.refreshAll(motorAVoltageSig, motorAVelocitySig, motorACurrentSig,
				motorBVoltageSig, motorBCurrentSig, motorBVelocitySig, intakePivotVoltageSig, intakePivotVelocitySig,
				intakePivotCurrentSig, intakePivotPositionSig);

		// Configuration
		inputs.motorAConnected = BaseStatusSignal.isAllGood(motorAVoltageSig);
		inputs.motorAConfigured = motorAConfigured;
		inputs.motorAVoltage = motorAVoltageSig.getValueAsDouble();
		inputs.motorARps = motorAVelocitySig.getValueAsDouble();
		inputs.motorASupplyCurrent = motorACurrentSig.getValueAsDouble();

		inputs.motorBConnected = BaseStatusSignal.isAllGood(motorBVoltageSig);
		inputs.motorBConfigured = motorBConfigured;
		inputs.motorBVoltage = motorBVoltageSig.getValueAsDouble();
		inputs.motorBRps = motorBVelocitySig.getValueAsDouble();
		inputs.motorBSupplyCurrent = motorBCurrentSig.getValueAsDouble();

		inputs.intakePivotMotorConnected = BaseStatusSignal.isAllGood(intakePivotVoltageSig);
		inputs.intakePivotMotorConfigured = pivotMotorConfigured;
		inputs.intakePivotMotorVoltage = intakePivotVoltageSig.getValueAsDouble();
		inputs.intakePivotMotorRps = intakePivotVelocitySig.getValueAsDouble();
		inputs.intakePivotMotorSupplyCurrent = intakePivotCurrentSig.getValueAsDouble();
		inputs.intakePivotMotorPosition = intakePivotPositionSig.getValueAsDouble();
	}

	@Override
	public void applyOutputs(IntakeIOOutputs outputs) {
		switch (outputs.rollerMode) {
			case OFF :
				motorA.stopMotor();
				motorB.stopMotor();
				break;
			case VOLTAGE :
				motorA.setControl(voltageOut.withOutput(outputs.rollerVoltageSetpoint).withEnableFOC(true));
				motorB.setControl(follower);
				break;
			default :
				break;
		}
		switch (outputs.pivotMode) {
			case OFF :
				pivotMotor.stopMotor();
				break;
			case POSITION :
				pivotMotor.setControl(pivotPosition.withPosition(outputs.pivotPositionSetpoint).withEnableFOC(false));
				break;
			case LOWER :
				pivotMotor.setControl(pivotPosition.withPosition(0.0).withEnableFOC(false));
				break;

			default :
				break;
		}
	}
}
