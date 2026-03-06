// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ramtech.frc2026.subsystems.shooter.hood;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import org.littletonrobotics.junction.Logger;
import org.ramtech.frc2026.Robot;
import org.ramtech.frc2026.subsystems.shooter.ShotCalculator;
import org.ramtech.frc2026.subsystems.shooter.hood.HoodIO.HoodIOOutputMode;
import org.ramtech.frc2026.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import org.ramtech.frc2026.subsystems.shooter.hood.HoodIO.HoodIOSetpointSource;
import org.ramtech.frc2026.util.ShooterSubsystem;

public class Hood extends ShooterSubsystem {
	private final Object outputsLock = new Object();

	// IO
	private final HoodIO io;
	private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
	private final HoodIOOutputs outputs = new HoodIOOutputs();
	// Alerts
	private final Debouncer hoodDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
	private final Alert hoodDisconnected = new Alert("Hood Angle Motor Disconnected!", Alert.AlertType.kWarning);

	/** Creates a new Hood. */
	public Hood(HoodIO io) {
		this.io = io;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		io.updateInputs(inputs);
		Logger.processInputs("Shooter/Hood", inputs);
		hoodDisconnected.set(Robot.showHardwareAlerts() && !hoodDebouncer.calculate(inputs.hoodConnected));
	}

	@Override
	public void periodicAfterScheduler() {
		synchronized (outputsLock) {
			Logger.recordOutput("Shooter/Hood/Mode", outputs.mode);
			Logger.recordOutput("Shooter/Hood/VoltageSetpoint", outputs.voltageSetpoint);
			Logger.recordOutput("Shooter/Hood/PositionSetpoint", outputs.positionSetpoint);
		}
	}

	@Override
	public void shooterPeriodic() {
		var shotCalculation = ShotCalculator.getInstance().getLatest();
		synchronized (outputsLock) {
			if (outputs.setpointSource == HoodIOSetpointSource.SHOT_CALCULATOR) {
				if (shotCalculation.isValid()) {
					outputs.mode = HoodIOOutputMode.POSITION;
					outputs.positionSetpoint = shotCalculation.hoodAngle();
				}
			}
			io.applyOutputs(outputs); // Set the targets for the motor
		}
	}

	public void enableCalculation() {
		synchronized (outputsLock) {
			outputs.setpointSource = HoodIOSetpointSource.SHOT_CALCULATOR;
		}
	}

	public void disableCalculation() {
		synchronized (outputsLock) {
			outputs.setpointSource = HoodIOSetpointSource.MANUAL;
		}
	}

	public void setVoltage(double voltage) {
		synchronized (outputsLock) {
			outputs.setpointSource = HoodIOSetpointSource.MANUAL;
			outputs.mode = HoodIOOutputMode.VOLTAGE;
			outputs.voltageSetpoint = voltage;
		}
	}

	public void setPosition(double position) {
		synchronized (outputsLock) {
			outputs.setpointSource = HoodIOSetpointSource.MANUAL;
			outputs.mode = HoodIOOutputMode.POSITION;
			outputs.positionSetpoint = position;
		}
	}

	public void stop() {
		synchronized (outputsLock) {
			outputs.setpointSource = HoodIOSetpointSource.MANUAL;
			outputs.mode = HoodIOOutputMode.OFF;
			outputs.voltageSetpoint = 0.0;
			outputs.positionSetpoint = 0.0;
		}
	}

	public double getHoodAngle() {
		return inputs.hoodPosition + ShotCalculator.hoodMinAngle;
	}
}
