// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ramtech.frc2026.commands;

import org.ramtech.frc2026.subsystems.intake.Intake;
import org.ramtech.frc2026.subsystems.shooter.turret.Turret;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LowerIntake extends Command {

	private final Intake intake;
	private final Turret turret;

	/** Creates a new LowerTurret. */
	public LowerIntake(Intake intake, Turret turret) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(intake, turret);
		this.intake = intake;
		this.turret = turret;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		turret.setTurretIntakeLock(true);
		intake.lowerPivot();

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		turret.setTurretIntakeLock(false);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return (intake.getPivotPosition() < 0.04);
	}
}
