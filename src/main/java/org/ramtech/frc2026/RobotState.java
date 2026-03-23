// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ramtech.frc2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.ramtech.frc2026.util.Zones;

/** Add your docs here. */
public class RobotState {
	private static RobotState instance;
	private Zones zones = new Zones();

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

	public double getHoodAngle() {
		return hoodAngleSupplier.get();
	}

	public Double getGyroAngleRate() {
		return gyroAngleRateSupplier.get();
	}

	/*
	 * Misc
	 */
	public void publishState() {
		Logger.recordOutput("RobotState/BaseRobotPose", RobotState.getInstance().getRobotPose());
		Logger.recordOutput("RobotState/BaseRobotRotation", RobotState.getInstance().getRotation());
		Logger.recordOutput("RobotState/ModuleStates", RobotState.getInstance().getModuleStates());
		Logger.recordOutput("RobotState/Acceleration", RobotState.getInstance().getAcceleration());
		Logger.recordOutput("RobotState/Zone", zones.getZoneFromPose(RobotState.getInstance().getRobotPose()));
		// SmartDashboard.putData(new );

		// Logger.recordOutput("RobotState/leftFarPass", TargetPoses.leftFarPass);
		// Logger.recordOutput("RobotState/rightFarPass", TargetPoses.rightFarPass);
		// Logger.recordOutput("RobotState/leftClosePass", TargetPoses.leftClosePass);
		// Logger.recordOutput("RobotState/rightClosePass", TargetPoses.rightClosePass);

	}
}
