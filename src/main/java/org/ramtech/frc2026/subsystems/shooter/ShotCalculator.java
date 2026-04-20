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
	 *            The angle of the hood in degrees from vertical (e.g. 11 to 52).
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
	private boolean shootingAllowed = true;
	private boolean hoodUnsafe = false;
	private boolean hoodEnableRequest = true; // on first enable, this should start everything up.
	public boolean transitionInProgress = false;
	public boolean switchCommanded = false;

	private Pose3d targetPose = TargetPoses.hub;
	private Zone zone = new Zone(0, 0, 0, 0, "Initialize", zoneType.nothing);

	private double angleRate = 0.0;

	private double swingVelocityX = 0.0;
	private double swingVelocityY = 0.0;

	private double turretDiffLast = 0.0;
	private Pose2d angleRatePose = new Pose2d();

	/*
	 * Constants
	 */
	private static final double shotCeiling = 2.8; // m
	private static final double rpsMin = 36;
	// private static final double rpsInterference = 80; // rps when the turret is
	// aiming at the polycarb at home
	private static final double rpsMax = 84;
	private static final double rpsIdeal = 76;
	private static final double rpsBump = 4;
	private static final double rpsMult = 1.13;

	// Angles from horizontal (Radians)
	public static final double hoodMinAngle = Math.toRadians(79);
	public static final double hoodRange = Math.toRadians(41);
	private static final double hoodMaxAngle = hoodMinAngle - hoodRange; // Math.toRadians(38)
	private static final double hoodIdealAngle = Math.toRadians(48);
	// private static final double hoodInterferenceAngle = Math.toRadians(55);

	private static final double g = 9.81; // m/s^2
	private static final double ballMass = 0.226796; // kg
	private static final double airDensity = 1.225; // kg/m^3
	private static final double dragCoeff = 0.47; // dimensionless

	private static final double ballDiameter = 0.15; // m
	private static final double ballArea = Math.PI * Math.pow(ballDiameter / 2, 2); // m^2
	private static final double ballCompression = 0.02; // m

	private static final double flyWheelDiam = 0.15; // m
	private static final double flyWheelCircum = flyWheelDiam * Math.PI; // m

	private static final double backWheelDiam = 0.0508; // m
	private static final double backWheelCircum = backWheelDiam * Math.PI; // m

	private static final double flyWheelRatio = (24.0 / 72.0); // relative to the motor
	private static final double backWheelRatio = (flyWheelRatio * (36.0 / 16.0) * (27.0 / 16.0));

	private static final double maxGyroVelocity = 6;
	private static final double maxChassisVelocity = 2.5;
	// private static final double maxGyroAcceleration = 15;
	private static double angleRateLast = 0;
	private static double tFlight = 0;

	/*
	 * Turret
	 */

	public Rotation2d getAngleToTarget(Pose3d turretPose, Pose3d targetPose) {
		var translationToTarget = targetPose.getTranslation().minus(turretPose.getTranslation());
		Rotation2d fieldAngleToTarget = new Rotation2d(translationToTarget.getX(), translationToTarget.getY());
		Rotation2d turretAngle = fieldAngleToTarget.minus(RobotState.getInstance().getRobotPose().getRotation());
		return turretAngle;
	}

	public double getTurretDistanceToTarget(Pose3d turretPose, Pose3d targetPose) {
		var translationToTarget = targetPose.getTranslation().minus(turretPose.getTranslation());
		return translationToTarget.getNorm();
	}

	public double getWrappedTurretAngle(double inputAngle) {
		double lastTarget = latest.turretAngle;
		double targetAngle = inputAngle;
		double finalTarget;

		double optomizedError = MathUtil.inputModulus(targetAngle - lastTarget, -180.0, 180.0);
		double unconstrainedTarget = lastTarget + optomizedError;

		if (unconstrainedTarget < TurretConstants.reverseSoftLimit - TurretConstants.turretAngleOffsetForZero) {
			switchCommanded = true;
			unconstrainedTarget += 360;
		} else if (unconstrainedTarget > TurretConstants.forwardSoftLimit - TurretConstants.turretAngleOffsetForZero) {
			unconstrainedTarget -= 360;
			switchCommanded = true;
		} else {
			switchCommanded = false;
		}

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

	public void update() {

		double tLoop = 0.005;
		double tOutput = 0.020;
		double tReact = 0.070;

		RobotState robotState = RobotState.getInstance();
		ChassisSpeeds fieldSpeeds = robotState.getFieldSpeeds();
		double gyroAngleRateRaw = robotState.getGyroAngleRate();
		double turretAngleFeedback = RobotState.getInstance().getTurretAngle();

		// Decoupled feedback logic
		// double safeHoodAngleFeedback = Math
		// .min((hoodMinAngle -
		// Math.toRadians(RobotState.getInstance().getHoodAngle())), hoodMinAngle);

		Pose2d robotPose = robotState.getRobotPose();
		Pose2d turretPose = robotPose.transformBy(
				new Transform2d(Offsets.turretOffset.getX(), Offsets.turretOffset.getY(), new Rotation2d()));

		Zone newZone = zoneUtil.getZoneFromPose(AllianceFlipUtil.apply(turretPose));

		angleRate = DataProcessing.sanitize(angleRateLast, -maxGyroVelocity, maxGyroVelocity, gyroAngleRateRaw);
		angleRateLast = angleRate;

		double fieldAngle = Math.toRadians(250 - 90) + robotState.getRotation().getRadians() + (angleRate * tLoop);
		double turretAngular = (Offsets.turretOffset.getTranslation().getNorm() * angleRate);

		Translation2d swingVelocityField = new Translation2d(0, new Rotation2d(fieldAngle));

		Translation2d fieldVelocity = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

		if (fieldVelocity.getNorm() > maxChassisVelocity) {
			fieldVelocity = fieldVelocity.times(maxChassisVelocity / fieldVelocity.getNorm());
		}

		Translation2d totalRobotVelocityField = fieldVelocity.plus(swingVelocityField);

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
				hoodUnsafe = true;
				shootingAllowed = false;
				break;
			case blockShooting :
				shootingAllowed = true;
				break;
			case nothing :
				break;
			default :
				break;
		}

		targetPose = AllianceFlipUtil.apply(targetPose);

		if (hoodEnableRequest == true && !(zone.type() == zoneType.hoodUnsafe)) {
			hoodUnsafe = false;
			hoodEnableRequest = false;
		} else {
			hoodEnableRequest = false;
		}

		double turretStaticToTarget = getTurretDistanceToTarget(new Pose3d(turretPose), targetPose);

		double vertExit = (Math.sin(last.hoodAngle) * ((ballDiameter + flyWheelDiam - ballCompression) / 2));
		double latExit = (Math.cos(last.hoodAngle) * ((ballDiameter + flyWheelDiam - ballCompression) / 2));

		double shotHeight = (TunerConstants.kWheelRadius.in(Meter) + 0.428625 + vertExit);
		double trajectoryCeiling = (shotCeiling - shotHeight);
		double vertFinal = (targetPose.getZ() - shotHeight);
		double latStatic = Math.max((turretStaticToTarget + latExit - 0.10795), 0.5);

		// --- Target-Relative Frame Transformation ---
		Rotation2d fieldAngleToTarget = new Rotation2d(targetPose.getX() - turretPose.getX(),
				targetPose.getY() - turretPose.getY());

		// 1. Rotate the robot's absolute velocity into the target's frame of reference.
		// X becomes velocity TOWARDS the target. Y becomes velocity PERPENDICULAR to
		// the target.
		Translation2d robotVelocityTargetFrame = totalRobotVelocityField.rotateBy(fieldAngleToTarget.unaryMinus());

		double robotVelocityTowards = robotVelocityTargetFrame.getX();
		double robotVelocityPerpendicular = robotVelocityTargetFrame.getY();

		// 1. Vertical velocity to reach the exact ceiling
		double verticalVelocity = Math.sqrt(2 * g * trajectoryCeiling);

		// 2. Time to reach the ceiling (apex)
		double timeToApex = verticalVelocity / g;

		// 3. Time to fall from the ceiling down to the target height
		double timeToTarget = Math.sqrt((2 * (trajectoryCeiling - vertFinal)) / g);

		// 4. Total flight time
		double totalFlightTime = Math.max(timeToApex + timeToTarget, 1);

		double velocityTowardsCombo = (latStatic / totalFlightTime) - robotVelocityTowards;

		Translation2d horizontalVelocityCombo = new Translation2d(velocityTowardsCombo, 0 - robotVelocityPerpendicular);

		double hoodAngle = Math.atan2(verticalVelocity, velocityTowardsCombo);

		Translation2d velocityVectorField = horizontalVelocityCombo.rotateBy(fieldAngleToTarget);

		double angleSub = hoodAngle - hoodIdealAngle;

		if (((rpsIdeal - last.flyWheelVelocity) / 50) < Math.abs(angleSub)) {
			if (angleSub < 0) {
				hoodAngle = hoodIdealAngle - ((rpsIdeal - last.flyWheelVelocity) / 50);
			} else {
				hoodAngle = hoodIdealAngle + ((rpsIdeal - last.flyWheelVelocity) / 50);
			}
		}

		// boolean turretInterference = (MathUtil.inputModulus(last.turretAngle, -180.0,
		// 180.0) >= -75.0
		// && MathUtil.inputModulus(last.turretAngle, -180.0, 180.0) <= 75.0);

		// if (turretInterference) {
		// if (hoodAngle < hoodInterferenceAngle) {
		// hoodAngle = hoodInterferenceAngle;
		// }
		// }

		// FIXED: Properly convert last.hoodAngle (vertical degrees) back to horizontal
		// radians before smoothing/sanitizing
		double lastHoodAngleRad = hoodMinAngle - Math.toRadians(last.hoodAngle);
		hoodAngle = DataProcessing.rawToSmooth(6, lastHoodAngleRad, hoodAngle);
		hoodAngle = DataProcessing.sanitize(lastHoodAngleRad, hoodMaxAngle, hoodMinAngle, hoodAngle);

		if (hoodUnsafe) {
			hoodAngle = hoodMinAngle;
		}

		// double velocityInitial = Math.hypot(velocityTowardsCombo, verticalVelocity);
		double velocityInitial = velocityTowardsCombo / Math.cos(hoodAngle);

		double airEst = (airDensity * ballArea * dragCoeff * velocityInitial * 0.5);
		double velocityTarget = velocityInitial
				* (1 + ((airEst * latStatic / (3 * ballMass * velocityInitial * Math.cos(hoodAngle)))));

		if (velocityInitial > velocityTarget) {
			velocityTarget = velocityInitial;
		}

		double flyWheelVelocity = ((velocityTarget
				/ (((flyWheelCircum * flyWheelRatio) + (backWheelCircum * backWheelRatio)) / 2)) * rpsMult) + rpsBump;

		// if (turretInterference) {
		// if (!Constants.isComp) {
		// flyWheelVelocity = Math.min(flyWheelVelocity, rpsInterference);
		// }
		// }
		flyWheelVelocity = DataProcessing.sanitize(last.flyWheelVelocity, rpsMin, rpsMax, flyWheelVelocity);

		Rotation2d dynamicRobotAngle = velocityVectorField.getAngle().minus(robotState.getRobotPose().getRotation());
		double turretAngle = getWrappedTurretAngle(dynamicRobotAngle.getDegrees())
				- Math.toDegrees((angleRate * tReact));

		// In horizontal radians, hoodMinAngle (79 deg) is the steep/retracted
		// safe position
		if (hoodUnsafe) {
			flyWheelVelocity = rpsMin;
		}

		boolean isValid = true;

		latest = new ShotParameters(isValid, Math.toDegrees(hoodMinAngle) - Math.toDegrees(hoodAngle),
				// to 52)
				flyWheelVelocity, 0.0, turretAngle, !hoodUnsafe, shootingAllowed);

		last = latest;

		double turretDiff = DataProcessing.rawToSmooth(6, turretDiffLast,
				Math.abs((turretAngle + 90) - turretAngleFeedback));
		turretDiffLast = turretDiff;
		transitionInProgress = ((turretDiff > 50) || switchCommanded);

		angleRatePose = robotPose.transformBy(new Transform2d(0.0, 0.0, new Rotation2d(angleRate * tOutput)));

	}

	public ShotParameters getLatest() {
		return latest;
	}

	public void publishShotParameters() {
		var params = getLatest();
		Logger.recordOutput("ShotCalculator/ShotParameters", params);
		Logger.recordOutput("ShotCalculator/TargetPose", targetPose);
		Logger.recordOutput("swingVelocityX", swingVelocityX);
		Logger.recordOutput("swingVelocityY", swingVelocityY);
		// Logger.recordOutput("anglerratepose", hoodAngle);
		// Logger.recordOutput("anglerratepose", safehood);
		Logger.recordOutput("anglerratepose", angleRatePose);
	}
}
