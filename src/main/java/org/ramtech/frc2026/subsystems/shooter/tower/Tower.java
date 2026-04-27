// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ramtech.frc2026.subsystems.shooter.tower;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import org.littletonrobotics.junction.Logger;
import org.ramtech.frc2026.Robot;
import org.ramtech.frc2026.subsystems.shooter.tower.TowerIO.TowerIOOutputMode;
import org.ramtech.frc2026.subsystems.shooter.tower.TowerIO.TowerIOOutputs;
import org.ramtech.frc2026.util.ShooterSubsystem;

public class Tower extends ShooterSubsystem {

	// IO
	private final TowerIO io;
	private final TowerIOInputsAutoLogged inputs = new TowerIOInputsAutoLogged();
	private final TowerIOOutputs outputs = new TowerIOOutputs();

	private final Object inputsLock = new Object(); // Create a lock
	// Alerts
	private final Debouncer towerMotorADebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
	private final Debouncer towerMotorBDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

	private final Alert towerMotorADisconnected = new Alert("Tower Motor Disconnected!", Alert.AlertType.kWarning);
	private final Alert towerMotorBDisconnected = new Alert("Tower Motor Disconnected!", Alert.AlertType.kWarning);

	/** Creates a new Tower. */
	public Tower(TowerIO io) {
		this.io = io;
	}

	public void periodic() {
		// This method will be called once per scheduler run
		synchronized (inputsLock) {
			// Logger.processInputs("Shooter/Tower", inputs);
		}
		towerMotorADisconnected
				.set(Robot.showHardwareAlerts() && !towerMotorADebouncer.calculate(inputs.towerMotorAConnected));
		towerMotorBDisconnected
				.set(Robot.showHardwareAlerts() && !towerMotorBDebouncer.calculate(inputs.towerMotorBConnected));

	}

	@Override
	public void periodicAfterScheduler() {
		Logger.recordOutput("Shooter/Tower/Mode", outputs.mode);
		Logger.recordOutput("Shooter/Tower/VoltageSetpoint", outputs.voltageSetpoint);
		Logger.recordOutput("Shoter/Tower/VelocitySetpoint", outputs.velocitySetpoint);
		io.applyOutputs(outputs); // Set the targets for the motor

	}

	public void setVoltage(double voltage) {
		outputs.mode = TowerIOOutputMode.VOLTAGE;
		outputs.voltageSetpoint = voltage;
	}

	public void setVelocity(double velocity) {
		outputs.mode = TowerIOOutputMode.VELOCITY;
		outputs.velocitySetpoint = velocity;
	}

	public void stop() {
		outputs.mode = TowerIOOutputMode.OFF;
		outputs.voltageSetpoint = 0.0;
		outputs.velocitySetpoint = 0.0;
	}

	@Override
	public void shooterPeriodic() {
		synchronized (inputsLock) {
			io.updateInputs(inputs);
		}
	}

	public double getVelocity() {
		return inputs.towerMotorAVelocity;
	}
}
