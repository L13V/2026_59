// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ramtech.frc2026.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import org.littletonrobotics.junction.Logger;
import org.ramtech.frc2026.Robot;
import org.ramtech.frc2026.subsystems.intake.IntakeIO.IntakeIOOutputMode;
import org.ramtech.frc2026.subsystems.intake.IntakeIO.IntakeIOOutputs;
import org.ramtech.frc2026.util.FullSubsystem;

public class Intake extends FullSubsystem {
	// IO
	private final IntakeIO io;
	private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
	private final IntakeIOOutputs outputs = new IntakeIOOutputs();
	// Alerts
	private final Debouncer rollerDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
	private final Alert rollerDisconnected = new Alert("Intake Roller Disconnected!", Alert.AlertType.kWarning);

	/** Creates a new Tower. */
	public Intake(IntakeIO io) {
		this.io = io;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);
		rollerDisconnected.set(Robot.showHardwareAlerts() && !rollerDebouncer.calculate(inputs.rollerConnected));
	}

	@Override
	public void periodicAfterScheduler() {
		io.applyOutputs(outputs); // Set the targets for the motor
		Logger.recordOutput("Intake/Roller/Mode", outputs.mode);
		Logger.recordOutput("Intake/Roller/Voltage", outputs.voltageSetpoint);
	}

	public void setVoltage(double voltage) {
		outputs.mode = IntakeIOOutputMode.VOLTAGE;
		outputs.voltageSetpoint = voltage;
	}

	public void setVelocity(double velocity) {
		outputs.mode = IntakeIOOutputMode.VELOCITY;
		outputs.velocitySetpoint = velocity;
	}

	public void stop() {
		outputs.mode = IntakeIOOutputMode.OFF;
		outputs.voltageSetpoint = 0.0;
		outputs.velocitySetpoint = 0.0;
	}
}
