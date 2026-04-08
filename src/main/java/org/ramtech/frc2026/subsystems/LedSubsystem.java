// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ramtech.frc2026.subsystems;

import static edu.wpi.first.units.Units.*;

import org.ramtech.frc2026.Constants.LedConstants;
import org.ramtech.frc2026.util.PhoenixUtil;
import org.ramtech.frc2026.RobotState;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
	final CANdle robotCANdle = new CANdle(LedConstants.CANdleID);

	// Slot 0: Tunnel
	// Slot 1: Turret
	// Slot 2:

	final SingleFadeAnimation shootingReady = new SingleFadeAnimation(13, 34).withSlot(2)
			.withColor(new RGBWColor(2, 255, 1, 0)).withFrameRate(Hertz.of(8.415));

	public LedSubsystem() {
		CANdleConfiguration config = new CANdleConfiguration();
		config.LED.BrightnessScalar = 1.0;
		config.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;

		PhoenixUtil.tryUntilOk(5, () -> robotCANdle.getConfigurator().apply(config));

	}

	@Override
	public void periodic() {
		if (DriverStation.isDisabled()) {
			robotCANdle.setControl(shootingReady);
		} else if (RobotState.getInstance().isTurretMisalinged()) {

		} else {

		}
	}
}
