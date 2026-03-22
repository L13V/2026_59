// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ramtech.frc2026.subsystems.shooter.turret;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import org.littletonrobotics.junction.Logger;
import org.ramtech.frc2026.Robot;
import org.ramtech.frc2026.subsystems.shooter.ShotCalculator;
import org.ramtech.frc2026.subsystems.shooter.turret.TurretIO.TurretIOOutputMode;
import org.ramtech.frc2026.subsystems.shooter.turret.TurretIO.TurretIOOutputs;
import org.ramtech.frc2026.subsystems.shooter.turret.TurretIO.TurretIOSetpointSource;
import org.ramtech.frc2026.util.ShooterSubsystem;

public class Turret extends ShooterSubsystem {
	private final Object outputsLock = new Object();

	// IO
	private final TurretIO io;
	private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
	private final TurretIOOutputs outputs = new TurretIOOutputs();
	// Alerts
	private final Debouncer turretDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

	private final Alert turretMotorDisconnected = new Alert("Turret Motor Disconnected!", Alert.AlertType.kWarning);
	private final Alert turretEncoderADisconnected = new Alert("Turret Encoder A Disconnected!",
			Alert.AlertType.kWarning);
	private final Alert turretEncoderBDisconnected = new Alert("Turret Encoder B Disconnected!",
			Alert.AlertType.kWarning);

	/** Creates a new Hood. */
	public Turret(TurretIO io) {
		this.io = io;
	}

	public void periodic() {
		// This method will be called once per scheduler run
		io.updateInputs(inputs);
		Logger.processInputs("Shooter/Turret", inputs);
		turretMotorDisconnected
				.set(Robot.showHardwareAlerts() && !turretDebouncer.calculate(inputs.turretMotorConnected));
		turretEncoderADisconnected
				.set(Robot.showHardwareAlerts() && !turretDebouncer.calculate(inputs.turretEncoderAConnected));
		turretEncoderBDisconnected
				.set(Robot.showHardwareAlerts() && !turretDebouncer.calculate(inputs.turretEncoderBConnected));
	}

	@Override
	public void periodicAfterScheduler() {
		synchronized (outputsLock) {
			Logger.recordOutput("Shooter/Turret/Mode", outputs.mode);
			Logger.recordOutput("Shooter/Turret/VoltageSetpoint", outputs.voltageSetpoint);
			Logger.recordOutput("Shooter/Turret/PositionSetpoint", outputs.positionSetpoint);
			Logger.recordOutput("Shooter/Turret/SetpointSource", outputs.setpointSource);
			Logger.recordOutput("Shooter/Turret/turretLockedByIntake", outputs.turretLockedByIntake);
			Logger.recordOutput("Shooter/Turret/turretLockedByDriver", outputs.turretLockedByDriver);

		}
	}

	@Override
	public void shooterPeriodic() {
		var shotCalculation = ShotCalculator.getInstance().getLatest();
		synchronized (outputsLock) {
			if (outputs.setpointSource == TurretIOSetpointSource.SHOT_CALCULATOR) {
				if (shotCalculation.isValid()) {
					outputs.mode = TurretIOOutputMode.POSITION;
					outputs.positionSetpoint = shotCalculation.turretAngle();
				}
			}
			io.applyOutputs(outputs); // Set the targets for the motor
		}
	}

	public boolean isIntakeLocked() {
		return outputs.turretLockedByIntake;
	}

	public boolean isDriverLocked() {
		return outputs.turretLockedByDriver;
	}

	public void enableCalculation() {
		synchronized (outputsLock) {
			outputs.setpointSource = TurretIOSetpointSource.SHOT_CALCULATOR;
		}
	}

	public void disableCalculation() {
		synchronized (outputsLock) {
			outputs.setpointSource = TurretIOSetpointSource.MANUAL;
		}
	}

	public void setVoltage(double voltage) {
		synchronized (outputsLock) {
			outputs.setpointSource = TurretIOSetpointSource.MANUAL;
			outputs.mode = TurretIOOutputMode.VOLTAGE;
			outputs.voltageSetpoint = voltage;
		}
	}

	public void setPosition(double position) {
		synchronized (outputsLock) {
			outputs.setpointSource = TurretIOSetpointSource.MANUAL;
			outputs.mode = TurretIOOutputMode.POSITION;
			outputs.positionSetpoint = position; // Applied offset for making zero straight
		}
	}

	public void setTurretIntakeLock(boolean lock) {
		synchronized (outputsLock) {
			outputs.turretLockedByIntake = lock;
		}
	}

	public void setTurretDriverLock(boolean lock) {
		synchronized (outputsLock) {
			outputs.turretLockedByDriver = lock;
		}
	}

	public void stop() {
		synchronized (outputsLock) {
			outputs.setpointSource = TurretIOSetpointSource.MANUAL;
			outputs.mode = TurretIOOutputMode.OFF;
			outputs.voltageSetpoint = 0.0;
			outputs.positionSetpoint = 0.0;
		}
	}

	public double getTurretAngle() {
		return inputs.TurretMotorPosition - 90;
	}
}
