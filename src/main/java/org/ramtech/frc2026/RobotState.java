// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ramtech.frc2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.ramtech.frc2026.subsystems.shooter.ShotCalculator;
import org.ramtech.frc2026.util.AllianceFlipUtil;

public class RobotState {
	private static RobotState instance;
	// private Zones zones = new Zones();

	public static enum GlobalStates {
		IDLE, INTAKING, SHOOTING
	}

	public GlobalStates globalState = GlobalStates.IDLE;

	public static RobotState getInstance() { //
		if (instance == null)
			instance = new RobotState();
		return instance;
	}

	// Create an initial pose for the supplier to overwrite.
	private Supplier<Pose2d> poseSupplier = Pose2d::new;
	private Supplier<ChassisSpeeds> speedSupplier = ChassisSpeeds::new;
	private Supplier<SwerveModuleState[]> moduleStateSupplier = () -> new SwerveModuleState[]{new SwerveModuleState(),
			new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
	private Supplier<double[]> accelerationSupplier = () -> new double[]{0.0, 0.0, 0.0};
	private Supplier<Double> flywheelRpsSupplier = () -> 0.0;
	private Supplier<Double> turretAngleSupplier = () -> 0.0;
	private Supplier<Double> hoodAngleSupplier = () -> 0.0;
	private Supplier<Double> gyroAngleRateSupplier = () -> 0.0;

	private Pose2d[] activePPTrajectory = new Pose2d[0];
	private Pose2d[] previewPPTrajectory = new Pose2d[0];

	Field2d field = new Field2d();

	/*
	 * Setting Suppliers
	 */
	public void setPoseSupplier(Supplier<Pose2d> supplier) {
		this.poseSupplier = supplier;
	}

	public void setSpeedSupplier(Supplier<ChassisSpeeds> supplier) {
		this.speedSupplier = supplier;
	}

	public void setModuleStateSupplier(Supplier<SwerveModuleState[]> supplier) {
		this.moduleStateSupplier = supplier;
	}

	public void setAccelerationSupplier(Supplier<double[]> supplier) {
		this.accelerationSupplier = supplier;
	}

	public void setFlywheelRpsSupplier(Supplier<Double> supplier) {
		this.flywheelRpsSupplier = supplier;
	}

	public void setTurretAngleSupplier(Supplier<Double> supplier) {
		this.turretAngleSupplier = supplier;
	}

	public void setHoodAngleSupplier(Supplier<Double> supplier) {
		this.hoodAngleSupplier = supplier;
	}

	public void setGyroAngleRateSupplier(Supplier<Double> supplier) {
		this.gyroAngleRateSupplier = supplier;
	}

	public void setActivePathPlannerTrajectory(Pose2d[] trajectory) {
		activePPTrajectory = trajectory;
	}

	public void setPreviewPathPlannerTrajectory(Pose2d[] trajectory) {
		previewPPTrajectory = trajectory;
	}
	/*
	 * Getters
	 */

	public Pose2d getRobotPose() {
		return poseSupplier.get();
	}

	public Rotation2d getRotation() {
		return poseSupplier.get().getRotation();
	}

	public ChassisSpeeds getChassisSpeeds() {
		return speedSupplier.get();
	}

	public ChassisSpeeds getFieldSpeeds() {
		return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
	}

	public SwerveModuleState[] getModuleStates() {
		return moduleStateSupplier.get();
	}

	public double[] getAcceleration() {
		return accelerationSupplier.get();
	}

	public double getFlywheelRps() {
		return flywheelRpsSupplier.get();
	}

	public double getTurretAngle() {
		return turretAngleSupplier.get();
	}

	public boolean isTurretMisalinged() {
		return (ShotCalculator.getInstance().transitionInProgress || ShotCalculator.getInstance().switchCommanded);
	}

	public double getHoodAngle() {
		return hoodAngleSupplier.get();
	}

	public Double getGyroAngleRate() {
		return gyroAngleRateSupplier.get();
	}

	public Double getBatteryVoltage() {
		return RobotController.getBatteryVoltage();
	}

	public void setGlobalState(GlobalStates newState) {
		globalState = newState;
	}

	public GlobalStates getGlobalState() {
		return globalState;
	}

	/*
	 * Misc
	 */
	public void publishState() {
		Pose2d robotpose = RobotState.getInstance().getRobotPose();

		Logger.recordOutput("RobotState/BaseRobotPose", robotpose);
		Logger.recordOutput("RobotState/GlobalState", globalState);
		// Logger.recordOutput("RobotState/BaseRobotRotation",
		// RobotState.getInstance().getRotation());
		// Logger.recordOutput("RobotState/ModuleStates",
		// RobotState.getInstance().getModuleStates());
		// Logger.recordOutput("RobotState/Acceleration",
		// RobotState.getInstance().getAcceleration());
		// Logger.recordOutput("RobotState/Zone",
		// zones.getZoneFromPose(RobotState.getInstance().getRobotPose()));

		field.setRobotPose(robotpose);

		if (DriverStation.isAutonomousEnabled()) {
			field.getObject("currentTrajectory").setPoses(AllianceFlipUtil.apply(activePPTrajectory));

		} else if (DriverStation.isDisabled()) {
			field.getObject("currentTrajectory").setPoses(AllianceFlipUtil.apply(previewPPTrajectory));
		} else {
			field.getObject("currentTrajectory").setPoses(new Pose2d[0]);
		}

		SmartDashboard.putData("Field", field);
		// Logger.recordOutput("RobotState/TurretPosition",
		// new Pose2d(getRobotPose().transformBy(Offsets.turretOffset2d).getX(),
		// getRobotPose().transformBy(Offsets.turretOffset2d).getY(),
		// new
		// Rotation2d(Units.degreesToRadians(ShotCalculator.getInstance().getLatest().turretAngle())
		// + getRobotPose().getRotation().getDegrees())));
		// SmartDashboard.putData(new );

		// Logger.recordOutput("RobotState/leftFarPass", TargetPoses.leftFarPass);
		// Logger.recordOutput("RobotState/rightFarPass", TargetPoses.rightFarPass);
		// Logger.recordOutput("RobotState/leftClosePass", TargetPoses.leftClosePass);
		// Logger.recordOutput("RobotState/rightClosePass", TargetPoses.rightClosePass);

	}
}
