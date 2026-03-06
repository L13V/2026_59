package org.ramtech.frc2026.subsystems.shooter.turret;

import static org.ramtech.frc2026.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.ramtech.frc2026.Constants;
import org.ramtech.frc2026.Constants.TurretConstants;

import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretIOReal implements TurretIO {
	// Motors
	private final TalonFX turretMotor = new TalonFX(TurretConstants.turretMotorId, Constants.CANivore); // Main Motor

	// Encoders
	private final CANcoder turretEncoderA = new CANcoder(TurretConstants.turretEncoderAId, Constants.CANivore);
	private final CANcoder turretEncoderB = new CANcoder(TurretConstants.turretEncoderBId, Constants.CANivore);

	// Configuration
	private final TalonFXConfiguration turretConfig = new TalonFXConfiguration();
	private final CANcoderConfiguration encoderAConfig = new CANcoderConfiguration();
	private final CANcoderConfiguration encoderBConfig = new CANcoderConfiguration();

	private boolean turretMotorConfigured = false;
	private boolean turretEncoderAConfigured = false;
	private boolean turretEncoderBConfigured = false;

	private boolean turretCrtComplete = false;

	// Status Signals
	private final StatusSignal<Voltage> turretMotorVoltageSig;
	private final StatusSignal<Angle> turretMotorPositionSig;
	private final StatusSignal<AngularVelocity> turretMotorVelocitySig;
	private final StatusSignal<Current> turretMotorCurrentSig;

	private final StatusSignal<Angle> turretEncoderAPositionSig;
	private final StatusSignal<Angle> turretEncoderBPositionSig;

	private final StatusSignal<Angle> turretEncoderAAbsPositionSig;

	private final StatusSignal<Angle> turretEncoderBAbsPositionSig;

	// Control Methods
	private final VoltageOut voltageOut = new VoltageOut(0); // Control Method
	private final MotionMagicVoltage MotionMagicVoltage = new MotionMagicVoltage(0);

	public final EasyCRT crt;
	public final EasyCRTConfig crtconfig;

	public TurretIOReal() {
		/*
		 * Turret Config
		 */
		// Base output stuff
		turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		// Converstion Factors
		turretConfig.Feedback.RotorToSensorRatio = TurretConstants.rotorToSensorRatio;
		turretConfig.Feedback.SensorToMechanismRatio = TurretConstants.SensorToMechanismRatio;
		// PID Motion Profile
		turretConfig.Slot0.kP = TurretConstants.kP_Slot0;
		turretConfig.Slot0.kI = TurretConstants.kI_Slot0;
		turretConfig.Slot0.kD = TurretConstants.kD_Slot0;
		turretConfig.Slot0.kS = TurretConstants.kS_Slot0;
		turretConfig.Slot0.kV = TurretConstants.kV_Slot0;
		turretConfig.Slot0.kA = TurretConstants.kA_Slot0;
		turretConfig.Slot0.kG = TurretConstants.kG_Slot0;
		turretConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.motionMagicAcceleration;
		turretConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.motionMagicCruiseVelocity;
		turretConfig.MotionMagic.MotionMagicJerk = TurretConstants.motionMagicJerk;
		// Current limits
		turretConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.StatorCurrentLimit;
		turretConfig.CurrentLimits.StatorCurrentLimitEnable = TurretConstants.StatorCurrentLimitEnable;
		turretConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants.SupplyCurrentLimit;
		turretConfig.CurrentLimits.SupplyCurrentLimitEnable = TurretConstants.SupplyCurrentLimitEnable;
		turretConfig.CurrentLimits.SupplyCurrentLowerLimit = TurretConstants.SupplyCurrentLowerLimit;
		turretConfig.CurrentLimits.SupplyCurrentLowerTime = TurretConstants.SupplyCurrentLowerTime;

		turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = TurretConstants.forwardSoftLimitEnable;
		turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.forwardSoftLimit;
		turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = TurretConstants.reverseSoftLimitEnable;
		turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.reverseSoftLimit;

		/*
		 * Encoder A
		 */
		encoderAConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		// encoderAConfig.MagnetSensor.MagnetOffset = 0.08349609375;
		encoderAConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
		/*
		 * Encoder B
		 */
		encoderBConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		// encoderBConfig.MagnetSensor.MagnetOffset = 0.130859375;
		encoderBConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

		/*
		 * Configure Motors
		 */
		turretMotorConfigured = tryUntilOkWithStatus(5, () -> turretMotor.getConfigurator().apply(turretConfig));
		turretEncoderAConfigured = tryUntilOkWithStatus(5,
				() -> turretEncoderA.getConfigurator().apply(encoderAConfig));
		turretEncoderBConfigured = tryUntilOkWithStatus(5,
				() -> turretEncoderB.getConfigurator().apply(encoderBConfig));

		/*
		 * Initalize Signals
		 */
		turretMotorVoltageSig = turretMotor.getMotorVoltage();
		turretMotorPositionSig = turretMotor.getPosition();
		turretMotorVelocitySig = turretMotor.getVelocity();
		turretMotorCurrentSig = turretMotor.getSupplyCurrent();
		turretEncoderAPositionSig = turretEncoderA.getPosition();
		turretEncoderBPositionSig = turretEncoderB.getPosition();

		turretEncoderAAbsPositionSig = turretEncoderA.getAbsolutePosition();
		turretEncoderBAbsPositionSig = turretEncoderB.getAbsolutePosition();

		BaseStatusSignal.setUpdateFrequencyForAll(50.0, turretMotorVoltageSig, turretMotorPositionSig,
				turretMotorVelocitySig, turretMotorCurrentSig, turretEncoderAPositionSig, turretEncoderBPositionSig,
				turretEncoderAAbsPositionSig, turretEncoderBAbsPositionSig);

		/*
		 * Optomize Bus Utilization
		 */
		turretMotor.optimizeBusUtilization();
		turretEncoderA.optimizeBusUtilization();
		turretEncoderB.optimizeBusUtilization();
		/*
		 * Liev's special area
		 */

		// Prep CRT Solver
		BaseStatusSignal.refreshAll(turretEncoderAAbsPositionSig, turretEncoderBAbsPositionSig);
		crtconfig = new EasyCRTConfig(turretEncoderAAbsPositionSig.asSupplier(),
				turretEncoderBAbsPositionSig.asSupplier()).withAbsoluteEncoder1GearingStages(148, 10)
						.withAbsoluteEncoder2GearingStages(148, 10, 16, 27).withAbsoluteEncoderInversions(true, true);

		crt = new EasyCRT(crtconfig);

		// attemptCrt(5);
	}

	public void attemptCrt(int attempts) { // Attempt Crt a couple times to
		// ensure minimal errors
		for (int i = 0; i < attempts; i++) { // Ratios
			// Refresh signals to ensure we have latest data for CRT
			BaseStatusSignal.refreshAll(turretEncoderAPositionSig, turretEncoderBPositionSig);
			// If everything online and configured, calculate start angle and push to motor.
			if (turretMotorConfigured & turretEncoderAConfigured & turretEncoderBConfigured) {
				// Solve and push to motor
				crt.getAngleOptional().ifPresent(mechAngle -> {
					turretCrtComplete = tryUntilOkWithStatus(5, () -> turretMotor.setPosition(mechAngle));
				});
				break; // if it worked, cancel the loop
			}
		}
	}

	public double returnangle() {
		BaseStatusSignal.refreshAll(turretEncoderAPositionSig, turretEncoderBPositionSig);
		return crt.getAngleOptional().map(mechAngle -> mechAngle.magnitude()).orElse(0.0);
	}

	@Override
	public void updateInputs(TurretIOInputs inputs) {
		inputs.turretMotorSignalOk = BaseStatusSignal.refreshAll(turretMotorVoltageSig, turretMotorPositionSig,
				turretMotorVelocitySig, turretMotorCurrentSig);
		inputs.turretEncoderASignalOk = BaseStatusSignal.refreshAll(turretEncoderAPositionSig);
		inputs.turretEncoderBSignalOk = BaseStatusSignal.refreshAll(turretEncoderBPositionSig);

		// Set Motor Connection Status
		inputs.turretMotorConnected = BaseStatusSignal.isAllGood(turretMotorVoltageSig);
		inputs.turretEncoderAConnected = BaseStatusSignal.isAllGood(turretEncoderAPositionSig);
		inputs.turretEncoderBConnected = BaseStatusSignal.isAllGood(turretEncoderBPositionSig);

		inputs.turretMotorConfigured = turretMotorConfigured;
		inputs.turretEncoderAConfigured = turretEncoderAConfigured;
		inputs.turretEncoderBConfigured = turretEncoderBConfigured;

		inputs.turretCrtComplete = turretCrtComplete;
		// Check and configure turret motor and encoders.
		// get motor outputs and push to TurretIO

		inputs.TurretMotorVoltage = turretMotorVoltageSig.getValueAsDouble();
		inputs.TurretMotorPosition = turretMotorPositionSig.getValueAsDouble();
		inputs.turretMotorVelocity = turretMotorVelocitySig.getValueAsDouble();
		inputs.TurretMotorSupplyCurrent = turretMotorCurrentSig.getValueAsDouble();

		inputs.TurretEncoderAPosition = turretEncoderAPositionSig.getValueAsDouble();
		inputs.TurretEncoderBPosition = turretEncoderBPositionSig.getValueAsDouble();
		inputs.TurretEncoderAAbsPosition = turretEncoderAAbsPositionSig.getValueAsDouble();
		inputs.TurretEncoderBAbsPosition = turretEncoderBAbsPositionSig.getValueAsDouble();

		inputs.crtValue = returnangle();
		inputs.crtStatus = crt.getLastStatus();

		// System.out.println("Satisfies: " + crtconfig.coverageSatisfiesRange());
		// System.out.println("Satisfies: " + crtconfig.getMatchTolerance());

	}

	@Override
	public void applyOutputs(TurretIOOutputs outputs) {
		switch (outputs.mode) {
			case OFF :
				turretMotor.stopMotor();
				break;
			case VOLTAGE :
				turretMotor.setControl(voltageOut.withOutput(outputs.voltageSetpoint).withEnableFOC(true));
				break;
			case POSITION :
				turretMotor.setControl(
						MotionMagicVoltage.withPosition(outputs.positionSetpoint + 90).withEnableFOC(true).withSlot(0));
				break;
		}
	}
}
