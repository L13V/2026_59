// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ramtech.frc2026.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import org.littletonrobotics.junction.Logger;
import org.ramtech.frc2026.Robot;
import org.ramtech.frc2026.subsystems.intake.IntakeIO.IntakeIORollerOutputMode;
import org.ramtech.frc2026.subsystems.intake.IntakeIO.IntakeIOOutputs;
import org.ramtech.frc2026.subsystems.intake.IntakeIO.IntakeIOPivotOutputMode;
import org.ramtech.frc2026.util.FullSubsystem;

public class Intake extends FullSubsystem {
	// IO
	private final IntakeIO io;
	private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
	private final IntakeIOOutputs outputs = new IntakeIOOutputs();
	// Alerts
	private final Debouncer motorADebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
	private final Debouncer motorBDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
	private final Debouncer intakePivotMotorDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

	private final Alert motorADisconnected = new Alert("Intake Motor A Disconnected!", Alert.AlertType.kWarning);
	private final Alert motorBDisconnected = new Alert("Intake Motor B Disconnected!", Alert.AlertType.kWarning);
	private final Alert intakePivotMotorDisconnected = new Alert("Intake Pivot Motor Disconnected!",
			Alert.AlertType.kWarning);

	/** Creates a new Tower. */
	public Intake(IntakeIO io) {
		this.io = io;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// io.updateInputs(inputs);
		// Logger.processInputs("Intake", inputs);
		motorADisconnected.set(Robot.showHardwareAlerts() && !motorADebouncer.calculate(inputs.motorAConnected));
		motorBDisconnected.set(Robot.showHardwareAlerts() && !motorBDebouncer.calculate(inputs.motorBConnected));
		intakePivotMotorDisconnected.set(
				Robot.showHardwareAlerts() && !intakePivotMotorDebouncer.calculate(inputs.intakePivotMotorConnected));

		if (outputs.pivotMode == IntakeIOPivotOutputMode.LOWER && inputs.intakePivotMotorPosition < 0.01) {
			// outputs.pivotMode = IntakeIOPivotOutputMode.OFF;
		}

	}

	@Override
	public void periodicAfterScheduler() {
		io.applyOutputs(outputs); // Set the targets for the motor
		Logger.recordOutput("Intake/Roller/Mode", outputs.rollerMode);
		Logger.recordOutput("Intake/Roller/VoltageSetpoint", outputs.rollerVoltageSetpoint);

		Logger.recordOutput("Intake/Pivot/Mode", outputs.pivotMode);
		Logger.recordOutput("Intake/Pivot/PositionSetpoint", outputs.pivotPositionSetpoint);
	}

	public double getPivotPosition() {
		return inputs.intakePivotMotorPosition;
	}

	public void setRollerVoltage(double voltage) {
		outputs.rollerMode = IntakeIORollerOutputMode.VOLTAGE;
		outputs.rollerVoltageSetpoint = voltage;
	}

	public void setPivotPosition(double position) {
		outputs.pivotMode = IntakeIOPivotOutputMode.POSITION;
		outputs.pivotPositionSetpoint = position;
	}

	public void lowerPivot() {
		outputs.pivotMode = IntakeIOPivotOutputMode.LOWER;
	}

	public void stopRollers() {
		outputs.rollerMode = IntakeIORollerOutputMode.OFF;
		outputs.rollerVoltageSetpoint = 0.0;
	}

	public void stopPivot() {
		outputs.pivotMode = IntakeIOPivotOutputMode.OFF;
	}
}
