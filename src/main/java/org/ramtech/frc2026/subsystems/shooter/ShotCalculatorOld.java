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
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.ramtech.frc2026.Constants;
import org.ramtech.frc2026.RobotState;
import org.ramtech.frc2026.generated.TunerConstants;

public class ShotCalculatorOld {
	private static ShotCalculatorOld instance = new ShotCalculatorOld();

	public static ShotCalculatorOld getInstance() {
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

	private double gyroAngleRateLast = 0.0;

	private double angleRate = 0.0;
	private double angleAccel = 0.0;

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

	private static final double flyWheelDiam = 0.153; // m
	private static final double flyWheelCircum = flyWheelDiam * Math.PI; // m

	private static final double backWheelDiam = 0.0508; // m
	private static final double backWheelCircum = backWheelDiam * Math.PI; // m

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
		// incorperate robot angle
		Rotation2d turretAngle = fieldAngleToTarget.minus(RobotState.getInstance().getRobotPose().getRotation());
		return turretAngle;
	}

	/**
	 * @return Get the normal distance from the turret to the target pose.
	 */
	public double getTurretDistanceToTarget(Pose3d turretPose, Pose3d targetPose) {
		// x and y translation to the center of the target
		var translationToTarget = targetPose.getTranslation().minus(turretPose.getTranslation());
		return translationToTarget.getNorm();
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

	public void update(double loopTime) {

		double tReact = 0.03 + loopTime;

		RobotState robotState = RobotState.getInstance();
		ChassisSpeeds chassisSpeeds = robotState.getChassisSpeeds();
		ChassisSpeeds fieldSpeeds = robotState.getFieldSpeeds();
		double gyroAngleRateRaw = robotState.getGyroAngleRate();

		// The turret's pose
		Pose2d robotPose = robotState.getRobotPose();

		Pose2d turretPose = robotPose.transformBy(
				new Transform2d(Offsets.turretOffset.getX(), Offsets.turretOffset.getY(), new Rotation2d()));

		// the current zone (may return null)
		Zone newZone = zoneUtil.getZoneFromPose(AllianceFlipUtil.apply(turretPose));

		// Standard derivative: (change in rate) / (change in time)
		double gyroRadAccel = (gyroAngleRateRaw - gyroAngleRateLast) / loopTime;

		gyroAngleRateLast = gyroAngleRateRaw;

		angleRate = DataProcessing.rawToSmooth(3, angleRate,
				(gyroAngleRateRaw * angleBias) + (chassisSpeeds.omegaRadiansPerSecond * (1 - angleBias))) * 1;

		angleAccel = DataProcessing.rawToSmooth(6, angleAccel,
				(gyroRadAccel * angleBias) + (gyroRadAccel * (1 - angleBias)));

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
				shootingAllowed = true;
				break;
			case blockShooting :
				shootingAllowed = true;
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

		// --- NEW: Clean WPILib future pose projection ---
		double futureTime = tFlightLast + tReact;

		// Predict the FUTURE pose of the chassis center
		double futureRobotX = robotPose.getX() + (fieldSpeeds.vxMetersPerSecond * futureTime);
		double futureRobotY = robotPose.getY() + (fieldSpeeds.vyMetersPerSecond * futureTime);

		// Project the chassis rotation forward using your smoothed angleRate
		double futureRobotAngle = robotPose.getRotation().getRadians() + (angleRate * futureTime);

		Pose2d futureRobotPose = new Pose2d(futureRobotX, futureRobotY, new Rotation2d(futureRobotAngle));
		dynamicPose = futureRobotPose;

		// Apply the turret offset to the FUTURE robot pose to automatically handle
		// tangential swinging
		Pose2d turretPoseDynamicXY = futureRobotPose.transformBy(Offsets.turretOffset2d);
		Pose3d turretPoseDynamic3d = new Pose3d(turretPoseDynamicXY);

		// Static pose for initial calculations
		Pose3d turretPoseStatic3d = new Pose3d(turretPose);
		// ------------------------------------------------

		double flywheelRpsFeedback = RobotState.getInstance().getFlywheelRps();
		double turretAngleFeedback = RobotState.getInstance().getTurretAngle();

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
		double angleSub = Math.toDegrees(hoodAngle) - hoodIdealAngle;

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

		if (turretInterference) { // Override angle
			if (hoodAngle < Math.toRadians(90 - hoodInterferenceAngle)) {
				hoodAngle = Math.toRadians(90 - hoodInterferenceAngle);
			}
		}

		hoodAngle = DataProcessing.rawToSmooth(7, Math.toRadians(90 - last.hoodAngle), hoodAngle);
		hoodAngle = DataProcessing.sanitize(Math.toRadians(90 - last.hoodAngle), Math.toRadians(90 - hoodMaxAngle),
				Math.toRadians(90 - hoodMinAngle), hoodAngle);

		double velocityInitial = (latFinal / Math.cos(hoodAngle))
				* Math.sqrt(g / (2 * ((latFinal * Math.tan(hoodAngle)) - vertFinal)));

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

		// Add chassis momentum to the ball's horizontal velocity
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
		Logger.recordOutput("anglerratepose", angleRatePose);
	}
}
