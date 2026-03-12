// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ramtech.frc2026.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Meter;

import org.littletonrobotics.junction.Logger;
import org.ramtech.frc2026.Constants.Offsets;
import org.ramtech.frc2026.Constants.TargetPoses;
import org.ramtech.frc2026.Constants.TurretConstants;
import org.ramtech.frc2026.generated.TunerConstants;
import org.ramtech.frc2026.util.DataProcessing;
import org.ramtech.frc2026.util.Zones;
import org.ramtech.frc2026.util.Zones.Zone;
import org.ramtech.frc2026.util.Zones.zoneType;
import org.ramtech.frc2026.RobotState;

public class ShotCalculator {
	private static ShotCalculator instance = new ShotCalculator();

	public static ShotCalculator getInstance() {
		return instance;
	}

	/**
	 * Represents the calculated parameters for a shot.
	 *
	 * @param isValid
	 *            Whether the shot calculation is valid and safe to execute.
	 * @param hoodAngle
	 *            The angle of the hood in degrees from horizontal.
	 * @param flyWheelVelocity
	 *            The target velocity of the flywheel in rotations per second (RPS).
	 * @param flyWheelFeedForward
	 *            The target feed forward the flywheel in rotations per second
	 *            (RPS).
	 * @param turretAngle
	 *            The target angle of the turret in degrees (Robot forward is 0
	 *            degrees).
	 * @param hoodSafe
	 *            Is the hood safe to raise?
	 * @param shootingAllowed
	 *            Is the robot allowed to index to shoot?
	 */
	public record ShotParameters(boolean isValid, double hoodAngle, double flyWheelVelocity, double flyWheelFeedForward,
			double turretAngle, boolean hoodSafe, boolean shootingAllowed) {
	}

	private volatile ShotParameters latest = new ShotParameters(false, 0, 0, 0, 0, false, false);
	private volatile ShotParameters last = new ShotParameters(false, 0, 0, 0, 0, false, false);

	private static final Zones zoneUtil = new Zones();
	private boolean shootingAllowed = false;
	private boolean hoodUnsafe = false;
	private boolean hoodEnableRequest = true; // on first enable, this should start everything up.
	private Pose3d targetPose = TargetPoses.hub;
	private Zone zone = new Zone(0, 0, 0, 0, "Initialize", zoneType.nothing);

	private double chassisAngleRateLast = 0.0;

	private double gyroAngleRateLast = 0.0;

	private double angleRate = 0.0;
	private double angleAccel = 0.0;

	private double angleBias = 0.8;
	private double turretAngleToRobot = Math.toRadians(250.0);

	private double tFlight = 0.0;
	private double tFlightLast = 0.0;

	private Pose2d dynamicPose = new Pose2d();

	/*
	 * Constants
	 */
	private static final double shotCeiling = 2.8; // m
	private static final double rpsMin = 30;
	private static final double rpsMax = 80;
	private static final double rpsBump = 0.7;
	private static final double rpsMult = 1.13;
	private static final double peakRPSS = 5000;

	public static final double hoodMinAngle = 10; // deg TODO: Verify
	private static final double hoodMaxAngle = 51; // deg
	private static final double hoodIdealAngle = 42;

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
	private static final double flyWheelRatio = (24.0 / 72.0); // relative to the motor
	private static final double backWheelRatio = (flyWheelRatio * (36.0 / 16.0) * (27.0 / 16.0)); // relative to the
																									// motor
	/*
	 * Turret
	 */

	/**
	 * @return The angle from the robots 0 degrees to a target pose.
	 */
	public Rotation2d getAngleToTarget(Pose3d startPose, Pose3d targetPose) {
		// x and y translation to the center of the target
		var translationToTarget = targetPose.getTranslation().minus(startPose.getTranslation());
		// top-down angle to target
		Rotation2d fieldAngleToTarget = new Rotation2d(translationToTarget.getX(), translationToTarget.getY());
		// Rotation2d fieldAngleToTarget = new Rotation2d(1, 1);
		// incorperate robot angle
		Rotation2d turretAngle = fieldAngleToTarget.minus(RobotState.getInstance().getRobotPose().getRotation());
		return turretAngle;
	}

	/**
	 * @return Get the normal distance from the turret to the target pose.
	 */
	public double getTurretDistanceToTarget(Pose3d turretPose, Pose3d targetPose) {
		// Pose3d robotpose = new Pose3d(RobotState.getInstance().getRobotPose());
		// Pose3d turretpose = robotpose.transformBy(Offsets.turretOffset); // to turret
		// and clockwise 90 degrees

		// x and y translation to the center of the target
		var translationToTarget = targetPose.getTranslation().minus(turretPose.getTranslation());
		return translationToTarget.getNorm();
	}

	/**
	 * @return A turret motor target angle closest to the current angle. 0 degrees
	 *         is the robot's front. The angle will need to be offset in order to
	 *         apply to the motor (-90)
	 */
	public double getClosestTurretAngleTargetToPose(Pose3d turretPose, Pose3d targetPose) {
		double lastTarget = latest.turretAngle;
		double targetAngle = getAngleToTarget(turretPose, targetPose).getDegrees();
		double finalTarget = 0;

		// Error within one rotation
		double optomizedError = MathUtil.inputModulus(targetAngle - lastTarget, -180.0, 180.0);
		// Closest target (NOT CONSTRAINED SO IT COULD DAMAGE THE MECHANISM)
		double unconstrainedTarget = lastTarget + optomizedError;
		// Check for illegal values
		if (unconstrainedTarget < TurretConstants.reverseSoftLimit - 90) { // -30
			unconstrainedTarget += 360;
		} else if (unconstrainedTarget > TurretConstants.forwardSoftLimit - 90) { // 510
			unconstrainedTarget -= 360;
		}

		// Final filter for safety
		finalTarget = MathUtil.clamp(unconstrainedTarget, TurretConstants.reverseSoftLimit - 90,
				TurretConstants.forwardSoftLimit - 90);
		return finalTarget;
	}

	public double getWrappedAngleForTurretMotorThingThatIAmTyring(double inputAngle) {
		double lastTarget = latest.turretAngle;
		double targetAngle = inputAngle;
		double finalTarget;

		// Error within one rotation
		double optomizedError = MathUtil.inputModulus(targetAngle - lastTarget, -180.0, 180.0);
		// Closest target (NOT CONSTRAINED SO IT COULD DAMAGE THE MECHANISM)
		double unconstrainedTarget = lastTarget + optomizedError;
		// Check for illegal values
		if (unconstrainedTarget < TurretConstants.reverseSoftLimit - 90) { // -30
			unconstrainedTarget += 360;
		} else if (unconstrainedTarget > TurretConstants.forwardSoftLimit - 90) { // 510
			unconstrainedTarget -= 360;
		}

		// Final filter for safety
		finalTarget = MathUtil.clamp(unconstrainedTarget, TurretConstants.reverseSoftLimit - 90,
				TurretConstants.forwardSoftLimit - 90);
		return finalTarget;
	}

	public void requestSafe() {
		hoodEnableRequest = true;
	}

	public void update(double loopTime) {
		double tReact = 0.05 + loopTime;

		RobotState robotState = RobotState.getInstance();
		ChassisSpeeds chassisSpeeds = robotState.getChassisSpeeds();
		ChassisSpeeds fieldSpeeds = robotState.getFieldSpeeds();
		double gyroAngleRateRaw = robotState.getGyroAngleRate();

		// The turret's pose
		Pose2d robotPose = robotState.getRobotPose();

		// Logger.recordOutput("hello2", robotPose);
		Pose2d turretPose = robotPose.transformBy(
				new Transform2d(Offsets.turretOffset.getX(), Offsets.turretOffset.getY(), new Rotation2d()));
		Pose3d turretPose3d = new Pose3d(robotPose).transformBy(Offsets.turretOffset);
		// the current zone (may return null)
		Zone newZone = zoneUtil.getZoneFromPose(turretPose);

		double chassisRadAccel = 0; // (chassisSpeeds.omegaRadiansPerSecond - chassisAngleRateLast) * (1000 /
									// loopTime);
		// chassisAngleRateLast = chassisSpeeds.omegaRadiansPerSecond; // store newest
		// value for the rate thing.

		double gyroRadAccel = 0;// (gyroAngleRateRaw - gyroAngleRateLast) * (1000 / loopTime);

		// gyroAngleRateLast = gyroAngleRateRaw;

		angleRate = DataProcessing.rawToSmooth(5, angleRate,
				(gyroAngleRateRaw * angleBias) + (chassisSpeeds.omegaRadiansPerSecond * (1 - angleBias)));

		// angleAccel = DataProcessing.rawToSmooth(5, angleAccel,
		// (gyroRadAccel * angleBias) + (gyroRadAccel * (1 - angleBias)));

		double shooterFieldAngle = Math.toRadians(250 - 90) + robotState.getRotation().getRadians()
				+ ((angleRate + (angleAccel / 2)) * tReact);

		double turretAngular = (Offsets.turretOffset.getTranslation().getNorm() * (angleRate + (angleAccel * tReact)));

		Translation2d velocityAdder = new Translation2d(turretAngular, new Rotation2d(shooterFieldAngle));

		// If the new zone is valid, overrite the old one.
		if (newZone != null) {
			zone = newZone;
		}
		zone = newZone != null ? newZone : zone;

		switch (zone != null ? zone.type() : zoneType.nothing) {
			case passingfarleft :
				shootingAllowed = true;
				targetPose = TargetPoses.leftFarPass;
				break;
			case passingfarright :
				shootingAllowed = true;
				targetPose = TargetPoses.rightFarPass;
				break;
			case passingcloseleft :
				shootingAllowed = true;
				targetPose = TargetPoses.leftClosePass;
				break;
			case passingcloseright :
				shootingAllowed = true;
				targetPose = TargetPoses.rightClosePass;
				break;
			case scoring :
				shootingAllowed = true;
				targetPose = TargetPoses.hub;
				break;
			case hoodUnsafe :
				hoodUnsafe = true;
				break;
			case blockShooting :
				shootingAllowed = false;
				break;
			case nothing :
				break;
			default :
				break;
		}

		if (hoodEnableRequest == true && !(zone.type() == zoneType.hoodUnsafe)) {
			hoodUnsafe = false;
			hoodEnableRequest = false;
		} else {
			hoodEnableRequest = false;
		}

		double flywheelRpsFeedback = RobotState.getInstance().getFlywheelRps();
		// double turretAngleFeedback = RobotState.getInstance().getTurretAngle();
		double hoodAngleFeedback = RobotState.getInstance().getHoodAngle();

		// double turretAngle = getClosestTurretAngleTargetToPose(turretPose3d,
		// targetPose);
		double turretDistanceToTarget = getTurretDistanceToTarget(turretPose3d, targetPose);

		double vertExit = (Math.sin(Math.toRadians(last.hoodAngle))
				* ((ballDiameter + flyWheelDiam - ballCompression) / 2));
		double latExit = (Math.cos(Math.toRadians(last.hoodAngle))
				* ((ballDiameter + flyWheelDiam - ballCompression) / 2));
		SmartDashboard.putNumber("vertExit", vertExit);
		SmartDashboard.putNumber("latExit", latExit);

		double shotHeight = (TunerConstants.kWheelRadius.in(Meter) + 0.428625 + vertExit);

		double trajectoryCeiling = (shotCeiling - shotHeight);

		double vertFinal = (targetPose.getZ() - shotHeight);

		double latFinal = (turretDistanceToTarget + latExit - 0.10795);

		double safeLatFinal = Math.max(0.001, turretDistanceToTarget + latExit - 0.10795);

		double hoodAngle = Math.atan(((2 * trajectoryCeiling)
				+ (2 * Math.sqrt(Math.max(0.0, trajectoryCeiling * (trajectoryCeiling - vertFinal)))))
				/ Math.max(0.001, latFinal));
		// TODO: Lookup table
		double angleSub = Math.toDegrees(hoodAngle) - hoodIdealAngle; // this is in degrees because its just
																		// comparing.

		if ((rpsMax - last.flyWheelVelocity) < Math.abs(angleSub)) { // TODO: Check
			if (angleSub < 0) {
				hoodAngle = Math.toRadians(hoodIdealAngle - (rpsMax - last.flyWheelVelocity));
			} else {
				hoodAngle = Math.toRadians(hoodIdealAngle + (rpsMax - last.flyWheelVelocity));
			}
		}
		SmartDashboard.putNumber("hoodAngle", hoodAngle);
		hoodAngle = DataProcessing.rawToSmooth(3, Math.toRadians(90 - last.hoodAngle), hoodAngle); // TODO: Check
		hoodAngle = DataProcessing.sanitize(Math.toRadians(90 - last.hoodAngle), Math.toRadians(90 - hoodMaxAngle),
				Math.toRadians(90 - hoodMinAngle), hoodAngle);
		// Insert lookup up table compariosn with the 1.2 (20%) range thing
		double velocityInitial = (latFinal / Math.max(0.001, Math.cos(hoodAngle)))
				* Math.sqrt(g / Math.max(0.001, 2 * ((latFinal * Math.tan(hoodAngle)) - vertFinal)));
		// double velocityInitial = 1;
		SmartDashboard.putNumber("velinit", velocityInitial);

		double velocityCombo = velocityInitial + Math.abs(0); // TODO: Implement robot pose stuff

		double airEst = (airDensity * ballArea * dragCoeff * velocityCombo * 0.5);

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
		double latDist = getTurretDistanceToTarget(turretPose3d, targetPose);
		tFlight = (latDist / LatVelocity)
				+ ((airEst * (Math.pow(latDist, 2))) / (2 * ballMass * Math.pow(LatVelocity, 2)));
		tFlight = DataProcessing.sanitize(tFlightLast, 1, 10, tFlight);

		tFlightLast = tFlight;

		double allVelocityX = (fieldSpeeds.vxMetersPerSecond + velocityAdder.getX()) * (tFlight + tReact);
		double allVelocityY = (fieldSpeeds.vyMetersPerSecond + velocityAdder.getY()) * (tFlight + tReact);

		Pose2d robotPoseDynamicXY = new Pose2d((robotPose.getX() + allVelocityX), (robotPose.getY() + allVelocityY),
				robotState.getRobotPose().getRotation());

		dynamicPose = robotPoseDynamicXY;
		Pose2d turretPoseDynamicXY = robotPoseDynamicXY.transformBy(Offsets.turretOffset2d);

		double turretAngle = getWrappedAngleForTurretMotorThingThatIAmTyring(
				getAngleToTarget(new Pose3d(turretPoseDynamicXY), targetPose).getDegrees()
						+ Math.toDegrees((angleRate + (angleAccel / 2)) * tReact));

		if ((rpsDiff) > peakRPSS * loopTime) {
			flyWheelFeedForward = rpsDiff / 100;
		}

		if (hoodUnsafe) {
			hoodAngle = Math.toRadians(90 - hoodMinAngle);
		}

		boolean isValid = true;
		latest = new ShotParameters(isValid, 90 - Math.toDegrees(hoodAngle), // Degrees FROM HORIZONTAL
				flyWheelVelocity, // Rps
				flyWheelFeedForward, turretAngle, // Degrees (Robot forward is 0 degrees)
				!hoodUnsafe, // True if the hood is safe
				shootingAllowed); // True if the robot is allowed to shoot in that area

		last = latest;
	}

	public ShotParameters getLatest() {
		return latest;
	}

	public void publishShotParameters() {
		var params = getLatest();
		Logger.recordOutput("ShotCalculator/ShotParameters", params);
		Logger.recordOutput("ShotCalculator/DynamicPose", dynamicPose);
		Logger.recordOutput("RobotState/ZoneName", zone != null ? zone.name() : "hello!");
		// Logger.recordOutput("ShotCalculator/AngleToTarget",
		// getTurretAngleToTarget().getDegrees());

	}
}
