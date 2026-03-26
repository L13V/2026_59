// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.ramtech.frc2026.util;

import org.ramtech.frc2026.subsystems.shooter.ShotCalculator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class HubShiftUtil {
	public enum ShiftEnum {
		TRANSITION, SHIFT1, SHIFT2, SHIFT3, SHIFT4, ENDGAME, AUTO, DISABLED;
	}

	public record ShiftInfo(ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {
	}

	private static Timer shiftTimer = new Timer();
	private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

	// --- ANTI-JITTER LATCH ---
	private static ShiftEnum highestTeleopShift = ShiftEnum.TRANSITION;

	private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
	private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

	private static final double minFuelCountDelay = 1.0;
	private static final double maxFuelCountDelay = 2.0;
	private static final double shiftEndFuelCountExtension = 3.0;

	public static final double autoEndTime = 20.0;
	public static final double teleopDuration = 140.0;
	private static final boolean[] activeSchedule = {true, true, false, true, false, true};
	private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};
	private static final double timeResetThreshold = 3.0;
	private static double shiftTimerOffset = 0.0;

	public static Alliance getFirstActiveAlliance() {
		var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

		// Return FMS value
		String message = DriverStation.getGameSpecificMessage();
		if (message.length() > 0) {
			char character = message.charAt(0);
			if (character == 'R') {
				return Alliance.Blue;
			} else if (character == 'B') {
				return Alliance.Red;
			}
		}

		// Return default value
		return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
	}

	/** Starts the timer at the beginning of teleop. */
	public static void initialize() {
		shiftTimerOffset = 0;
		highestTeleopShift = ShiftEnum.TRANSITION; // Reset the latch at the start of the match
		shiftTimer.restart();
	}

	private static boolean[] getSchedule() {
		boolean[] currentSchedule;
		Alliance startAlliance = getFirstActiveAlliance();
		currentSchedule = startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
				? activeSchedule
				: inactiveSchedule;
		return currentSchedule;
	}

	private static ShiftInfo getShiftInfo(boolean[] currentSchedule, double[] startTimes, double[] endTimes) {
		double timerValue = shiftTimer.get();
		double currentTime = timerValue - shiftTimerOffset;
		double stateTimeElapsed = currentTime;
		double stateTimeRemaining = 0.0;
		boolean active = false;
		ShiftEnum currentShift = ShiftEnum.DISABLED;
		double fieldTeleopTime = 140.0 - DriverStation.getMatchTime();

		if (DriverStation.isAutonomousEnabled()) {
			stateTimeElapsed = currentTime;
			stateTimeRemaining = autoEndTime - currentTime;
			active = true;
			currentShift = ShiftEnum.AUTO;
		} else if (DriverStation.isEnabled()) {
			// Adjust the current offset if the time difference is above the threshold
			if (Math.abs(fieldTeleopTime - currentTime) >= timeResetThreshold && fieldTeleopTime <= 135
					&& DriverStation.isFMSAttached()) {
				shiftTimerOffset = timerValue - fieldTeleopTime;
				currentTime = fieldTeleopTime;
			}

			int currentShiftIndex = -1;
			for (int i = 0; i < startTimes.length; i++) {
				if (currentTime >= startTimes[i] && currentTime < endTimes[i]) {
					currentShiftIndex = i;
					break;
				}
			}
			if (currentShiftIndex < 0) {
				// After last shift, so assume endgame
				currentShiftIndex = startTimes.length - 1;
			}

			// Calculate elapsed and remaining time in the current shift, ignoring combined
			// shifts
			stateTimeElapsed = currentTime - startTimes[currentShiftIndex];
			stateTimeRemaining = endTimes[currentShiftIndex] - currentTime;

			// If the state is the same as the last shift, combine the elapsed time
			if (currentShiftIndex > 0) {
				if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
					stateTimeElapsed = currentTime - startTimes[currentShiftIndex - 1];
				}
			}

			// If the state is the same as the next shift, combine the remaining time
			if (currentShiftIndex < endTimes.length - 1) {
				if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
					stateTimeRemaining = endTimes[currentShiftIndex + 1] - currentTime;
				}
			}

			active = currentSchedule[currentShiftIndex];
			currentShift = shiftsEnums[currentShiftIndex];
		}
		return new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
	}

	public static ShiftInfo getOfficialShiftInfo() {
		return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
	}

	public static ShiftInfo getShiftedShiftInfo() {
		boolean[] shiftSchedule = getSchedule();

		// 1. Get current flight time safely
		double tFlight = Math.max(0.0, ShotCalculator.getInstance().gettFlight());

		// 2. Calculate dynamic fudges based on the physical reality of the hub
		// Start early so fuel arrives and processes right as it opens
		double dynApproachingFudge = -1.0 * (tFlight + minFuelCountDelay);

		// Stop early enough that fuel flies and processes before the grace period ends
		double dynEndingFudge = shiftEndFuelCountExtension - (tFlight + maxFuelCountDelay);

		double[] shiftedStarts;
		double[] shiftedEnds;

		// 3. Map the fudges to the correct boundaries
		if (shiftSchedule[1] == true) { // Starting active
			shiftedStarts = new double[]{0.0, 10.0, 35.0 + dynEndingFudge, 60.0 + dynApproachingFudge,
					85.0 + dynEndingFudge, 110.0 + dynApproachingFudge};
			shiftedEnds = new double[]{10.0, 35.0 + dynEndingFudge, 60.0 + dynApproachingFudge, 85.0 + dynEndingFudge,
					110.0 + dynApproachingFudge, 140.0};
		} else { // Starting inactive
			shiftedStarts = new double[]{0.0, 10.0 + dynEndingFudge, 35.0 + dynApproachingFudge, 60.0 + dynEndingFudge,
					85.0 + dynApproachingFudge, 110.0};
			shiftedEnds = new double[]{10.0 + dynEndingFudge, 35.0 + dynApproachingFudge, 60.0 + dynEndingFudge,
					85.0 + dynApproachingFudge, 110.0, 140.0};
		}

		// 4. Get the raw shift info based on our dynamic times
		ShiftInfo rawInfo = getShiftInfo(shiftSchedule, shiftedStarts, shiftedEnds);

		// 5. ANTI-JITTER LOCK: Prevent the state from ever going backwards during
		// Teleop
		ShiftEnum current = rawInfo.currentShift();

		// We only lock states during Teleop (ignore AUTO and DISABLED)
		if (current != ShiftEnum.AUTO && current != ShiftEnum.DISABLED) {

			if (current.ordinal() > highestTeleopShift.ordinal()) {
				highestTeleopShift = current; // Latch to the new, higher state
			} else if (current.ordinal() < highestTeleopShift.ordinal()) {
				// We are latched. Recalculate time remaining/elapsed based on the latched
				// bounds
				int latchedIndex = highestTeleopShift.ordinal();
				boolean latchedActive = shiftSchedule[latchedIndex];

				double currentTime = shiftTimer.get() - shiftTimerOffset;
				double latchedElapsed = currentTime - shiftedStarts[latchedIndex];
				double latchedRemaining = shiftedEnds[latchedIndex] - currentTime;

				// Apply combining logic for the latched state
				if (latchedIndex > 0 && shiftSchedule[latchedIndex] == shiftSchedule[latchedIndex - 1]) {
					latchedElapsed = currentTime - shiftedStarts[latchedIndex - 1];
				}
				if (latchedIndex < shiftedEnds.length - 1
						&& shiftSchedule[latchedIndex] == shiftSchedule[latchedIndex + 1]) {
					latchedRemaining = shiftedEnds[latchedIndex + 1] - currentTime;
				}

				return new ShiftInfo(highestTeleopShift, latchedElapsed, latchedRemaining, latchedActive);
			}
		}

		return rawInfo;
	}
}
