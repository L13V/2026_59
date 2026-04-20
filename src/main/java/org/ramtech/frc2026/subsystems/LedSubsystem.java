// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ramtech.frc2026.subsystems;

import static edu.wpi.first.units.Units.*;

import org.ramtech.frc2026.Constants.LedConstants;
import org.ramtech.frc2026.RobotState.GlobalStates;
import org.ramtech.frc2026.util.PhoenixUtil;
import org.ramtech.frc2026.Constants;
import org.ramtech.frc2026.RobotState;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
	final CANdle robotCANdle = new CANdle(LedConstants.CANdleID, Constants.CANivore);

	// State tracking so we only update the CANdle when necessary
	private GlobalStates lastGlobalState = null;
	private boolean lastMisalignedState = false;
	private boolean lastDisabledState = false;

	// ==========================================
	// SLOT 0: TUNNEL ANIMATIONS (LEDs 0 to 13)
	// ==========================================
	private final SingleFadeAnimation tunnelDisabled = new SingleFadeAnimation(8, 13).withSlot(0)
			.withColor(new RGBWColor(0, 0, 255, 0)).withFrameRate(Hertz.of(100.0));
	private final SingleFadeAnimation tunnelReady = new SingleFadeAnimation(8, 13).withSlot(0)
			.withColor(new RGBWColor(255, 255, 255, 255)).withFrameRate(Hertz.of(100));

	private final ColorFlowAnimation tunnelShooting = new ColorFlowAnimation(8, 13).withSlot(0)
			.withColor(new RGBWColor(10, 1, 254, 0)).withDirection(AnimationDirectionValue.Forward)
			.withFrameRate(Hertz.of(200));

	// ==========================================
	// SLOT 1: TURRET ANIMATIONS (LEDs 14 to 24)
	// ==========================================
	private final SingleFadeAnimation turretDisabled = new SingleFadeAnimation(14, 24).withSlot(1)
			.withColor(new RGBWColor(0, 0, 255, 0)).withFrameRate(Hertz.of(100.0));
	private final SingleFadeAnimation turretReady = new SingleFadeAnimation(14, 24).withSlot(1)
			.withColor(new RGBWColor(2, 255, 1, 0)).withFrameRate(Hertz.of(100.0));

	private final StrobeAnimation turretMisaligned = new StrobeAnimation(14, 24).withSlot(1)
			.withColor(new RGBWColor(255, 0, 0, 0)).withFrameRate(Hertz.of(300.0));

	private final SingleFadeAnimation turretShooting = new SingleFadeAnimation(14, 24).withSlot(1)
			.withColor(new RGBWColor(255, 191, 0, 0)).withFrameRate(Hertz.of(100));

	// ==========================================
	// SLOT 2: UNDERGLOW ANIMATIONS (LEDs 25 to 34)
	// ==========================================
	private final SingleFadeAnimation underglowDisabled = new SingleFadeAnimation(25, 34).withSlot(2)
			.withColor(new RGBWColor(0, 0, 255, 0)).withFrameRate(Hertz.of(100.0));
	private final SingleFadeAnimation underglowReady = new SingleFadeAnimation(25, 34).withSlot(2)
			.withColor(new RGBWColor(2, 255, 1, 0)) // Green Fade
			.withFrameRate(Hertz.of(100.0));

	private final StrobeAnimation underglowMisaligned = new StrobeAnimation(25, 34).withSlot(2)
			.withColor(new RGBWColor(255, 0, 0, 0)) // Red Strobe
			.withFrameRate(Hertz.of(300.0));

	private final ColorFlowAnimation underglowShooting = new ColorFlowAnimation(25, 34).withSlot(2)
			.withColor(new RGBWColor(255, 191, 0, 0)) // Orange Flow
			.withDirection(AnimationDirectionValue.Forward).withFrameRate(Hertz.of(100));

	public LedSubsystem() {
		CANdleConfiguration config = new CANdleConfiguration();
		config.LED.BrightnessScalar = 1.0;
		config.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
		config.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.On;
		config.LED.StripType = StripTypeValue.RGB;

		PhoenixUtil.tryUntilOk(5, () -> robotCANdle.getConfigurator().apply(config));
	}

	@Override
	public void periodic() {
		boolean currentDisabled = DriverStation.isDisabled();
		GlobalStates currentGlobalState = RobotState.getInstance().getGlobalState();
		boolean currentMisaligned = RobotState.getInstance().isTurretMisalinged();

		// Only send setControl if one of our states has actually changed
		if (currentDisabled != lastDisabledState || currentGlobalState != lastGlobalState
				|| currentMisaligned != lastMisalignedState) {

			if (currentDisabled) {
				robotCANdle.setControl(tunnelDisabled);
				robotCANdle.setControl(turretDisabled);
				robotCANdle.setControl(underglowDisabled);

			} else if (currentGlobalState == GlobalStates.SHOOTING) {
				robotCANdle.setControl(tunnelShooting);
				robotCANdle.setControl(turretShooting);
				robotCANdle.setControl(underglowShooting);

			} else if (currentMisaligned) {
				robotCANdle.setControl(tunnelReady);
				robotCANdle.setControl(turretMisaligned);
				robotCANdle.setControl(underglowMisaligned);

			} else {
				// Default "Ready" state while enabled and aligned
				robotCANdle.setControl(tunnelReady);
				robotCANdle.setControl(turretReady);
				robotCANdle.setControl(underglowReady);
			}

			// Update our trackers
			lastDisabledState = currentDisabled;
			lastGlobalState = currentGlobalState;
			lastMisalignedState = currentMisaligned;
		}
	}
}
