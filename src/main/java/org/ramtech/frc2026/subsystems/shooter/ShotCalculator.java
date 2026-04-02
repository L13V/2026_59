package org.ramtech.frc2026.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import org.ramtech.frc2026.Constants.Offsets;
import org.ramtech.frc2026.Constants.TargetPoses;
import org.ramtech.frc2026.Constants.TurretConstants;
import org.ramtech.frc2026.util.AllianceFlipUtil;
import org.ramtech.frc2026.util.DataProcessing;
import org.ramtech.frc2026.util.HubShiftUtil;
import org.ramtech.frc2026.util.Zones;
import org.ramtech.frc2026.util.Zones.Zone;
import org.ramtech.frc2026.util.Zones.zoneType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.ramtech.frc2026.Constants;
import org.ramtech.frc2026.RobotState;
import org.ramtech.frc2026.generated.TunerConstants;

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
	 *            The target anturretAngleOffsetForZerogle of the turret in degrees
	 *            (Robot forward is 0 degrees).
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
	private boolean shootingAllowed = true;
	private boolean hoodUnsafe = false;
	private boolean hoodEnableRequest = true; // on first enable, this should start everything up.
	public boolean transitionInProgress = false;
	public boolean switchCommanded = false;

	private Pose3d targetPose = TargetPoses.hub;
	private Zone zone = new Zone(0, 0, 0, 0, "Initialize", zoneType.nothing);

	// private double chassisAngleRateLast = 0.0;

	private double gyroAngleRateLast = 0.0;

	private double angleRate = 0.0;
	private double angleAccel = 0.0;

	// NEW: 1. Calculate the dynamic pose FIRST using tFlightLast
	private double velocityAdderX = 0.0;
	private double velocityAdderY = 0.0;

	private double angleBias = 0.8;

	private double tFlight = 0.0;
	private double tFlightLast = 0.0;

	private double turretDiffLast = 0.0;

	private Pose2d dynamicPose = new Pose2d();
	private Pose2d angleRatePose = new Pose2d();

	/*
	 * Constants
	 */
	private static final double shotCeiling = 2.8; // m
	private static final double rpsMin = 30;
	private static final double rpsInterference = 65; // rps when the turret is gonna hit the polycarb
	private static final double rpsMax = 84;
	public static double defaultRpsBump = -6.0;

	// private static double rpsBump = Preferences.getDouble("rpsBump",
	// defaultRpsBump);
	private static double rpsBump = -6;
	private static final double rpsMult = 1.27;
	private static final double peakRPSS = 5000;

	public static final double hoodMinAngle = 11; // deg TODO: Verify
	private static final double hoodMaxAngle = 52; // deg
	private static final double hoodIdealAngle = 42;
	private static final double hoodInterferenceAngle = 32;

	private static final double g = 9.81; // m/s^2
	private static final double ballMass = 0.226796; // kg
	private static final double airDensity = 1.225; // kg/m^3
	private static final double dragCoeff = 0.47; // dimensionless

	private static final double ballDiameter = 0.15; // m
	private static final double ballArea = Math.PI * Math.pow(ballDiameter / 2, 2); // m^2
	private static final double ballCompression = 0.02; // m

	// private static final double lookupTollerance = 1.2; // *100%

	private static final double flyWheelDiam = 0.153; // m
	private static final double flyWheelCircum = flyWheelDiam * Math.PI; // m

	private static final double backWheelDiam = 0.0508; // m
	private static final double backWheelCircum = backWheelDiam * Math.PI; // m

	// private static final double angleLookup = 0; // TODO: MAKE it a lookup
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
	// public double getClosestTurretAngleTargetToPose(Pose3d turretPose, Pose3d
	// targetPose) {
	// double lastTarget = latest.turretAngle;
	// double targetAngle = getAngleToTarget(turretPose, targetPose).getDegrees();
	// double finalTarget = 0;

	// // Error within one rotation
	// double optomizedError = MathUtil.inputModulus(targetAngle - lastTarget,
	// -180.0, 180.0);
	// // Closest target (NOT CONSTRAINED SO IT COULD DAMAGE THE MECHANISM)
	// double unconstrainedTarget = lastTarget + optomizedError;
	// // Check for illegal values
	// if (unconstrainedTarget < TurretConstants.reverseSoftLimit -
	// TurretConstants.turretAngleOffsetForZero) {
	// unconstrainedTarget += 360;
	// } else if (unconstrainedTarget > TurretConstants.forwardSoftLimit -
	// TurretConstants.turretAngleOffsetForZero) {
	// unconstrainedTarget -= 360;
	// }

	// // Final filter for safety
	// finalTarget = MathUtil.clamp(unconstrainedTarget,
	// TurretConstants.reverseSoftLimit - TurretConstants.turretAngleOffsetForZero,
	// TurretConstants.forwardSoftLimit - TurretConstants.turretAngleOffsetForZero);
	// return finalTarget;
	// }

	public double getWrappedAngleForTurretMotorThingThatIAmTyring(double inputAngle) {
		double lastTarget = latest.turretAngle;
		double targetAngle = inputAngle;
		double finalTarget;

		// Error within one rotation
		double optomizedError = MathUtil.inputModulus(targetAngle - lastTarget, -180.0, 180.0);
		// Closest target (NOT CONSTRAINED SO IT COULD DAMAGE THE MECHANISM)
		double unconstrainedTarget = lastTarget + optomizedError;
		// Check for illegal values
		if (unconstrainedTarget < TurretConstants.reverseSoftLimit - TurretConstants.turretAngleOffsetForZero) { // -30
			switchCommanded = true;
			unconstrainedTarget += 360;
		} else if (unconstrainedTarget > TurretConstants.forwardSoftLimit - TurretConstants.turretAngleOffsetForZero) { // 510
			unconstrainedTarget -= 360;
			switchCommanded = true;
		} else {
			switchCommanded = false;
		}
		// Final filter for safety
		finalTarget = MathUtil.clamp(unconstrainedTarget,
				TurretConstants.reverseSoftLimit - TurretConstants.turretAngleOffsetForZero,
				TurretConstants.forwardSoftLimit - TurretConstants.turretAngleOffsetForZero);
		return finalTarget;
	}

	public void setHoodUnsafe() {
		hoodUnsafe = true;
	}

	public void requestSafe() {
		hoodEnableRequest = true;
	}

	public double gettFlight() {
		return tFlight;
	}

	public void refreshRpsBump() {
		// rpsBump = Preferences.getDouble("rpsBump", rpsBump);
	}

	// public double getRps

	public void update(double loopTime) {

		double tReact = 0.03 + loopTime;
		// double tReact = 0.035;

		RobotState robotState = RobotState.getInstance();
		ChassisSpeeds chassisSpeeds = robotState.getChassisSpeeds();
		ChassisSpeeds fieldSpeeds = robotState.getFieldSpeeds();
		double gyroAngleRateRaw = robotState.getGyroAngleRate();

		// The turret's pose
		Pose2d robotPose = robotState.getRobotPose();

		Pose2d turretPose = robotPose.transformBy(
				new Transform2d(Offsets.turretOffset.getX(), Offsets.turretOffset.getY(), new Rotation2d()));
		// Pose3d turretPose3d = new
		// Pose3d(robotPose).transformBy(Offsets.turretOffset);
		// the current zone (may return null)
		Zone newZone = zoneUtil.getZoneFromPose(AllianceFlipUtil.apply(turretPose));

		double chassisRadAccel = 0; // (chassisSpeeds.omegaRadiansPerSecond - chassisAngleRateLast) * (1000 /
									// loopTime);
		// chassisAngleRateLast = chassisSpeeds.omegaRadiansPerSecond; // store newest
		// value for the rate thing.

		// Standard derivative: (change in rate) / (change in time)
		double gyroRadAccel = (gyroAngleRateRaw - gyroAngleRateLast) / loopTime;

		gyroAngleRateLast = gyroAngleRateRaw;

		angleRate = DataProcessing.rawToSmooth(3, angleRate,
				(gyroAngleRateRaw * angleBias) + (chassisSpeeds.omegaRadiansPerSecond * (1 - angleBias))) * 1;

		angleAccel = DataProcessing.rawToSmooth(6, angleAccel,
				(gyroRadAccel * angleBias) + (gyroRadAccel * (1 - angleBias)));

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
				shootingAllowed = HubShiftUtil.getShiftedShiftInfo().active();
				targetPose = TargetPoses.hub;
				break;
			case hoodUnsafe :
				// hoodUnsafe = true;
				shootingAllowed = true;
				break;
			case blockShooting :
				shootingAllowed = true; // TODO: liev dorfman fix this please.
				break;
			case nothing :
				break;
			default :
				break;
		}

		// flip target pose
		targetPose = AllianceFlipUtil.apply(targetPose);

		if (hoodEnableRequest == true && !(zone.type() == zoneType.hoodUnsafe)) {
			hoodUnsafe = false;
			hoodEnableRequest = false;
		} else {
			hoodEnableRequest = false;
		}
		// SmartDashboard.putNumber("VelocityAdderX", velocityAdder.getX());
		// SmartDashboard.putNumber("VelocityAdderY", velocityAdder.getY());

		double velocityAdderX = velocityAdder.getX();
		double velocityAdderY = velocityAdder.getY();
		// NEW: 1. Calculate the dynamic pose FIRST using tFlightLast
		double allVelocityX = (fieldSpeeds.vxMetersPerSecond + velocityAdderX) * (tFlightLast + tReact);
		double allVelocityY = (fieldSpeeds.vyMetersPerSecond + velocityAdderY) * (tFlightLast + tReact);

		Pose2d robotPoseDynamicXY = new Pose2d((robotPose.getX() + allVelocityX), (robotPose.getY() + allVelocityY),
				robotState.getRobotPose().getRotation());
		dynamicPose = robotPoseDynamicXY;

		Pose2d turretPoseDynamicXY = robotPoseDynamicXY.transformBy(Offsets.turretOffset2d);

		Pose3d turretPoseDynamic3d = new Pose3d(turretPoseDynamicXY); // We need this for the 3D distance check

		Pose3d turretPoseStatic3d = new Pose3d(turretPose);

		double flywheelRpsFeedback = RobotState.getInstance().getFlywheelRps();
		double turretAngleFeedback = RobotState.getInstance().getTurretAngle();

		// double hoodAngleFeedback = RobotState.getInstance().getHoodAngle();

		// double turretAngle = getClosestTurretAngleTargetToPose(turretPose3d,
		// targetPose);
		double turretDynamicToTarget = getTurretDistanceToTarget(turretPoseDynamic3d, targetPose);
		double turretStaticToTarget = getTurretDistanceToTarget(turretPoseStatic3d, targetPose);

		double vertExit = (Math.sin(Math.toRadians(last.hoodAngle))
				* ((ballDiameter + flyWheelDiam - ballCompression) / 2));
		double latExit = (Math.cos(Math.toRadians(last.hoodAngle))
				* ((ballDiameter + flyWheelDiam - ballCompression) / 2));

		double shotHeight = (TunerConstants.kWheelRadius.in(Meter) + 0.428625 + vertExit);

		double trajectoryCeiling = (shotCeiling - shotHeight);

		double vertFinal = (targetPose.getZ() - shotHeight);

		double latFinal = (turretDynamicToTarget + latExit - 0.10795);

		double latStatic = (turretStaticToTarget + latExit - 0.10795);

		double hoodAngle = Math
				.atan(((2 * trajectoryCeiling) + (2 * Math.sqrt((trajectoryCeiling * (trajectoryCeiling - vertFinal)))))
						/ Math.max(0.001, latFinal));
		double angleSub = Math.toDegrees(hoodAngle) - hoodIdealAngle; // this is in degrees because its just
																		// comparing.

		if ((rpsMax - last.flyWheelVelocity) < Math.abs(angleSub)) {
			if (angleSub < 0) {
				hoodAngle = Math.toRadians(hoodIdealAngle - (rpsMax - last.flyWheelVelocity));
			} else {
				hoodAngle = Math.toRadians(hoodIdealAngle + (rpsMax - last.flyWheelVelocity));
			}
		}
		// Is the turret going to be blocked?

		boolean turretInterference = (MathUtil.inputModulus(last.turretAngle, -180.0, 180.0) >= -75.0
				&& MathUtil.inputModulus(last.turretAngle, -180.0, 180.0) <= 155.0);

		// boolean turretInterference = (MathUtil.inputModulus(last.turretAngle, 0, 360)
		// > -75
		// && MathUtil.inputModulus(last.turretAngle, 0, 360) < 150);

		if (turretInterference) { // Override angle
			// System.out.println("Override");
			if (hoodAngle < Math.toRadians(90 - hoodInterferenceAngle)) {
				// System.out.println("Overrid2");

				hoodAngle = Math.toRadians(90 - hoodInterferenceAngle);
			}
		}

		// SmartDashboard.putNumber("hoodAngle", hoodAngle);
		hoodAngle = DataProcessing.rawToSmooth(7, Math.toRadians(90 - last.hoodAngle), hoodAngle);
		hoodAngle = DataProcessing.sanitize(Math.toRadians(90 - last.hoodAngle), Math.toRadians(90 - hoodMaxAngle),
				Math.toRadians(90 - hoodMinAngle), hoodAngle);
		// Insert lookup up table compariosn with the 1.2 (20%) range thing
		double velocityInitial = (latFinal / Math.cos(hoodAngle))
				* Math.sqrt(g / (2 * ((latFinal * Math.tan(hoodAngle)) - vertFinal)));

		// SmartDashboard.putNumber("velinit", velocityInitial);

		// NEW: 3. Add the chassis momentum to the initial velocity requirement

		// double velocityCombo = velocityInitial + Math.abs(Math.hypot(allVelocityX,
		// allVelocityY));

		double airEst = (airDensity * ballArea * dragCoeff * velocityInitial * 0.5);

		double velocityTarget = velocityInitial
				* (1 + ((airEst * latFinal) / (3 * ballMass * velocityInitial * Math.cos(hoodAngle))));

		if (velocityInitial > velocityTarget) {
			velocityTarget = velocityInitial;
		}
		double flyWheelVelocity = ((velocityTarget
				/ (((flyWheelCircum * flyWheelRatio) + (backWheelCircum * backWheelRatio)) / 2)) * rpsMult) + rpsBump;

		// Is the turret going to be blocked?
		if (turretInterference) {
			if (!Constants.isComp) { // Not on comp field
				flyWheelVelocity = Math.min(flyWheelVelocity, rpsInterference);
			}
		}
		flyWheelVelocity = DataProcessing.sanitize(last.flyWheelVelocity, rpsMin, rpsMax, flyWheelVelocity);

		double rpsDiff = flyWheelVelocity - flywheelRpsFeedback;
		double flyWheelFeedForward = 0;

		// Translation2d translationToTarget =
		// targetPose.getTranslation().toTranslation2d()
		// .minus(robotPose.getTranslation());
		// Rotation2d directionToTarget = translationToTarget.getAngle();

		double velocityInitStatic = Math.min((latStatic / Math.cos(hoodAngle))
				* Math.sqrt(g / (2 * ((latStatic * Math.tan(hoodAngle)) - vertFinal))), velocityTarget);

		double LatVelocity = velocityInitStatic
				* (1 + ((airEst * latStatic) / (3 * ballMass * velocityInitStatic * Math.cos(hoodAngle))))
				* Math.cos(hoodAngle);

		// --- NEW: Factor in chassis radial velocity towards the target ---
		double dx = targetPose.getX() - turretPoseStatic3d.getX();
		double dy = targetPose.getY() - turretPoseStatic3d.getY();
		double distance = Math.hypot(dx, dy);

		double dirX = dx / distance;
		double dirY = dy / distance;

		// Dot product to find how fast the chassis is moving directly towards the
		// target
		double vRadial = (fieldSpeeds.vxMetersPerSecond * dirX) + (fieldSpeeds.vyMetersPerSecond * dirY);

		// Add chassis momentum to the ball's horizontal velocity (prevent
		// divide-by-zero or negative flight times)
		double actualLatVelocity = Math.max(0.1, LatVelocity + vRadial);

		// 4. Calculate tFlight using the TRUE field-relative velocity
		tFlight = (latStatic / actualLatVelocity)
				+ ((airEst * Math.pow(latStatic, 2)) / (2 * ballMass * Math.pow(actualLatVelocity, 2)));
		// ---------------------------------------

		tFlightLast = tFlight;

		double turretAngle = getWrappedAngleForTurretMotorThingThatIAmTyring(
				getAngleToTarget(new Pose3d(turretPoseDynamicXY), targetPose).getDegrees()
						- Math.toDegrees((angleRate + (angleAccel / 2)) * (tReact)));
		if ((rpsDiff) > peakRPSS * loopTime) {
			flyWheelFeedForward = rpsDiff / 100;
		}

		if (hoodUnsafe) {
			hoodAngle = Math.toRadians(90 - hoodMinAngle);
			flyWheelVelocity = rpsMin;
		}

		boolean isValid = true;

		latest = new ShotParameters(isValid, 90 - Math.toDegrees(hoodAngle), // Degrees FROM HORIZONTAL
				flyWheelVelocity, // Rps
				flyWheelFeedForward, turretAngle, // Degrees (Robot forward is 0 degrees)
				!hoodUnsafe, // True if the hood is safe
				shootingAllowed); // True if the robot is allowed to shoot in that area

		last = latest;

		double turretDiff = DataProcessing.rawToSmooth(6, turretDiffLast,
				Math.abs((turretAngle + 90) - turretAngleFeedback));
		turretDiffLast = turretDiff;

		transitionInProgress = ((turretDiff > 60) || switchCommanded);

		// angleRatePose = robotPose.rotateBy(new Rotation2d(angleRate));
		angleRatePose = robotPose.transformBy(new Transform2d(0.0, 0.0,
				new Rotation2d(Math.toRadians(1 * Math.toDegrees((angleRate + (angleAccel / 2)) * (tReact))))));

	}

	public ShotParameters getLatest() {
		return latest;
	}

	public void publishShotParameters() {
		var params = getLatest();
		Logger.recordOutput("ShotCalculator/ShotParameters", params);
		Logger.recordOutput("ShotCalculator/DynamicPose", dynamicPose);
		Logger.recordOutput("ShotCalculator/tFlight", tFlight);
		Logger.recordOutput("ShotCalculator/TargetPose", targetPose);
		// Logger.recordOutput("switch orsometihn", switchCommanded);
		// Logger.recordOutput("tranny orsometihn", transitionInProgress);
		Logger.recordOutput("VelocityAdderX", velocityAdderX);
		Logger.recordOutput("VelocityAdderY", velocityAdderY);
		// SmartDashboard.putData("");

		Logger.recordOutput("anglerratepose", angleRatePose);

		// Logger.recordOutput("ShotCalculator/AngleToTarget",
		// getTurretAngleToTarget().getDegrees());

	}
}
