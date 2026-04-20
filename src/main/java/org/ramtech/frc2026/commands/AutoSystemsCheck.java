package org.ramtech.frc2026.commands;

import org.ramtech.frc2026.Constants.HoodConstants;
import org.ramtech.frc2026.Constants.TurretConstants;
import org.ramtech.frc2026.subsystems.LedSubsystem;
import org.ramtech.frc2026.subsystems.drive.Drive;
import org.ramtech.frc2026.subsystems.indexer.Indexer;
import org.ramtech.frc2026.subsystems.intake.Intake;
import org.ramtech.frc2026.subsystems.shooter.flywheel.Flywheel;
import org.ramtech.frc2026.subsystems.shooter.hood.Hood;
import org.ramtech.frc2026.subsystems.shooter.tower.Tower;
import org.ramtech.frc2026.subsystems.shooter.turret.Turret;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class AutoSystemsCheck extends SequentialCommandGroup {
	public AutoSystemsCheck(Drive drive, Turret turret, Hood hood, Flywheel flywheel, Intake intake, Tower tower,
			Indexer indexer, LedSubsystem leds) {
		addCommands(new InstantCommand(() -> {
			turret.disableCalculation();
			flywheel.stop();
			hood.disableCalculation();
		}), new StartEndCommand(() -> drive.runVelocity(new ChassisSpeeds(0.5, 0.0, 0.0)), () -> drive.stop(), drive)
				.withTimeout(1.5),
				turret.run(() -> turret.setSlowSystemCheckPosition(TurretConstants.forwardSoftLimit))
						.until(() -> turret.atPositionSetpoint(TurretConstants.forwardSoftLimit)).withTimeout(3.0),
				turret.run(() -> turret.setSlowSystemCheckPosition(TurretConstants.reverseSoftLimit))
						.until(() -> turret.atPositionSetpoint(TurretConstants.reverseSoftLimit)).withTimeout(3.0),
				hood.run(() -> hood.setSlowSystemCheckPosition(HoodConstants.forwardSoftLimit))
						.until(() -> hood.atPositionSetpoint(HoodConstants.forwardSoftLimit)).withTimeout(0.5),
				hood.run(() -> hood.setSlowSystemCheckPosition(HoodConstants.reverseSoftLimit))
						.until(() -> hood.atPositionSetpoint(HoodConstants.reverseSoftLimit)).withTimeout(0.5),
				flywheel.run(() -> flywheel.setVelocity(20)).withTimeout(1.0), flywheel.runOnce(() -> flywheel.stop()),
				tower.run(() -> tower.setVoltage(5)).withTimeout(1.0).finallyDo(() -> tower.stop()),
				indexer.run(() -> indexer.setVoltage(5)).withTimeout(1.0).finallyDo(() -> indexer.stop()),
				intake.run(() -> intake.setPivotPosition(0.1)).withTimeout(1.0),
				intake.run(() -> intake.lowerPivot()).withTimeout(0.25),
				intake.run(() -> intake.setRollerVoltage(5)).withTimeout(1.0).finallyDo(() -> intake.stopRollers()));
	}
}
