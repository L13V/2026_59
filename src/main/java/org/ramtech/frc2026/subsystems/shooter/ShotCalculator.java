// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ramtech.frc2026.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Meter;

import org.littletonrobotics.junction.Logger;
import org.ramtech.frc2026.Constants.Offsets;
import org.ramtech.frc2026.Constants.TargetPoses;
import org.ramtech.frc2026.Constants.TurretConstants;
import org.ramtech.frc2026.generated.TunerConstants;
import org.ramtech.frc2026.util.DataProcessing;
import org.ramtech.frc2026.RobotState;

public class ShotCalculator {
	private static ShotCalculator instance = new ShotCalculator();

	public static ShotCalculator getInstance() {
		return instance;
	}

	/**
	 * Represents the calculated parameters for a shot.
	 * 
	 * @param isValid          Whether the shot calculation is valid and safe to
	 *                         execute.
	 * @param hoodAngle        The angle of the hood in degrees from horizontal.
	 * @param flyWheelVelocity The target velocity of the flywheel in rotations per
	 *                         second (RPS).
	 * @param towerVelocity    The target velocity of the tower in rotations per
	 *                         second (RPS).
	 * @param turretAngle      The target angle of the turret in degrees (Robot
	 *                         forward is 0 degrees).
	 */
	public record ShotParameters(boolean isValid, double hoodAngle, double flyWheelVelocity, double towerVelocity,
			double turretAngle) {
	}

	private volatile ShotParameters latest = new ShotParameters(false, 0, 0, 0, 0);
	private volatile ShotParameters last = new ShotParameters(false, 0, 0, 0, 0);;

	/*
	 * Constants
	 */
	private static final double shotCeiling = 2.5; // m
	private static final double rpsMin = 30;
	private static final double rpsMax = 111;
	private static final double rpsBump = 4;
	private static final double rpsMult = 1.0;
	private static final double peakRPSS = 5000;

	public static final double hoodMinAngle = 12; // deg TODO: Verify
	private static final double hoodMaxAngle = 53; // deg

	private static final double g = 9.83069; // m/s^2
	private static final double ballMass = 0.226796; // kg
	private static final double airDensity = 1.225; // kg/m^3
	private static final double dragCoeff = 0.47; // dimensionless

	private static final double ballDiameter = 0.15; // m
	private static final double ballArea = Math.PI * Math.pow(ballDiameter / 2, 2); // m^2
	private static final double ballCompression = 0.02; // m

	private static final double lookupTollerance = 1.2; // *100% // TODO: MAKE TUNABLE

	private static final double flyWheelDiam = 0.156; // m
	private static final double flyWheelCircum = flyWheelDiam * Math.PI; // m

	private static final double backWheelDiam = 0.0508; // m
	private static final double backWheelCircum = backWheelDiam * Math.PI; // m

	private static final double angleLookup = 0; // TODO: MAKE it a lookup
	private static final double flyWheelRatio = 24 / 72; // relative to the motor
	private static final double backWheelRatio = flyWheelRatio * (36 / 16) * (27 / 16); // relative to the motor
	/*
	 * Turret
	 */

	/**
	 * @return The distance from the turret's zero to the target
	 *         hub pose.
	 */
	public Rotation2d getTurretAngleToHub() {
		Pose3d robotpose = new Pose3d(RobotState.getInstance().getRobotPose());
		Pose3d turretpose = robotpose.transformBy(Offsets.turretOffset); // to turret and clockwise 90 degrees

		// x and y translation to the center of the hub
		var translationToHub = TargetPoses.hub.getTranslation().minus(turretpose.getTranslation());
		// top-down angle to hub
		Rotation2d fieldAngleToHub = new Rotation2d(translationToHub.getX(), translationToHub.getY());
		// incorperate robot angle
		Rotation2d turretAngle = fieldAngleToHub.minus(RobotState.getInstance().getRobotPose().getRotation());
		return turretAngle;
	}

	/**
	 * @return The angle within one rotation from the turret's origin (robot front =
	 *         0 degrees) to the target
	 *         hub pose.
	 */
	public double getTurretDistanceToTarget() {
		Pose3d robotpose = new Pose3d(RobotState.getInstance().getRobotPose());
		Pose3d turretpose = robotpose.transformBy(Offsets.turretOffset); // to turret and clockwise 90 degrees

		// x and y translation to the center of the hub
		var translationToHub = TargetPoses.hub.getTranslation().minus(turretpose.getTranslation());
		return translationToHub.getNorm();
	}

	/*
	 * Calculate Nearest Real Target Angle
	 */
	public double getClosestTurretTarget() {
		double lastTarget = latest.turretAngle;
		double targetAngle = getTurretAngleToHub().getDegrees();
		double finalTarget = 0;

		// Error within one rotation
		double optomizedError = MathUtil.inputModulus(targetAngle - lastTarget, -180.0, 180.0);
		// Closest target (NOT CONSTRAINED SO IT COULD DAMAGE THE MECHANISM)
		double unconstrainedTarget = lastTarget + optomizedError;
		// Check for illegal values
		if (unconstrainedTarget < TurretConstants.reverseSoftLimit - 90) {
			unconstrainedTarget += 360;
		} else if (unconstrainedTarget > TurretConstants.forwardSoftLimit - 90) {
			unconstrainedTarget -= 360;
		}

		// Final filter for safety
		finalTarget = MathUtil.clamp(unconstrainedTarget, TurretConstants.reverseSoftLimit - 90,
				TurretConstants.forwardSoftLimit - 90);
		return finalTarget;
	}

	public void update(double loopTime) {
		double flywheelRpsFeedback = RobotState.getInstance().getFlywheelRps();
		double turretAngleFeedback = RobotState.getInstance().getTurretAngle();
		double hoodAngleFeedback = RobotState.getInstance().getHoodAngle();


		double turretAngle = getClosestTurretTarget();
		double turretDistanceToTarget = getTurretDistanceToTarget();

		/*
		 * Fish
		 */
		// DataProcessing.sanitize(last.hoodAngle, hoodMinAngle, hoodMaxAngle,);
		double sanetizedHoodAngle = Units.degreesToRadians(last.hoodAngle); // TODO: Use feedback

		double vertExit = (Math.sin(sanetizedHoodAngle) * ((ballDiameter + flyWheelDiam - ballCompression) / 2));
		SmartDashboard.putNumber("VertExit", vertExit);
		double latExit = (Math.cos(sanetizedHoodAngle) * ((ballDiameter + flyWheelDiam - ballCompression) / 2));
		SmartDashboard.putNumber("LatExit", latExit);

		double shotHeight = (TunerConstants.kWheelRadius.in(Meter) + 0.428625 + vertExit);

		double trajectoryCeiling = (shotCeiling - shotHeight);

		double vertFinal = (TargetPoses.hub.getZ() - shotHeight);
		SmartDashboard.putNumber("VertFinal", vertFinal);

		double latFinal = (turretDistanceToTarget + latExit - 0.10795);
		SmartDashboard.putNumber("LatFinal", latFinal);

		double hoodAngle = (Math
				.atan(((2 * trajectoryCeiling) + (2 * Math.sqrt(trajectoryCeiling * (trajectoryCeiling - vertFinal))))
						/ latFinal)); // TODO: Log (This is the normal)
		hoodAngle = DataProcessing.sanitize(Math.toRadians(last.hoodAngle), Math.toRadians(hoodMinAngle),
				Math.toRadians(hoodMaxAngle), hoodAngle);

		// Insert lookup up table compariosn with the 1.2 (20%) range thing

		double velocityInitial = (latFinal / Math.cos(last.hoodAngle))
				* Math.sqrt(g / 2 * ((latFinal * Math.tan(hoodAngle)) - vertFinal));

		double airEst = (airDensity * ballArea * dragCoeff * velocityInitial * 0.5);

		double velocityTarget = velocityInitial
				* (1 + ((airEst * latFinal) / (3 * ballMass * velocityInitial * Math.cos(hoodAngle))));

		if (velocityInitial > velocityTarget) {
			velocityTarget = velocityInitial;
		}

		double flyWheelVelocity = ((velocityTarget
				/ (((flyWheelCircum * flyWheelRatio) + (backWheelCircum * backWheelRatio)) / 2)) * rpsMult) + rpsBump;

		double LatVelocity = velocityTarget * Math.cos(hoodAngle);

		flyWheelVelocity = DataProcessing.sanitize(last.flyWheelVelocity, rpsMin, rpsMax, flyWheelVelocity);

		double rpsDiff = flyWheelVelocity - flywheelRpsFeedback; // replace last with feedback

		double flyWheelFeedForward = 0;

		if ((rpsDiff) > peakRPSS * loopTime) {
			flyWheelFeedForward = rpsDiff / 100;
		}

		// double flyWheelVelocity = 40.0;
		double towerVelocity = 40.0; // TODO: Remove
		boolean isValid = true;
		latest = new ShotParameters(isValid, 90 - Math.toDegrees(hoodAngle), // Degrees FROM HORIZONTAL
				flyWheelVelocity, // Rps
				40.0, // Rps
				turretAngle); // Degrees (Robot forward is 0 degrees)

		last = latest;
	}

	public ShotParameters getLatest() {
		return latest;
	}

	public void publishShotParameters() {
		var params = getLatest();
		Logger.recordOutput("ShotCalculator/ShotParameters", params);
		Logger.recordOutput("ShotCalculator/AngleToHub", getTurretAngleToHub().getDegrees());

	}
}
