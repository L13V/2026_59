package org.ramtech.frc2026.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class DynamicRateLimiter2D {
	private Translation2d m_prevVal;
	private double m_prevTime;

	/**
	 * @param initialValue
	 *            The value to start at (usually 0)
	 */
	public DynamicRateLimiter2D(Translation2d initialValue) {
		m_prevVal = initialValue;
		m_prevTime = Timer.getFPGATimestamp();
	}

	/**
	 * Calculates the limited value based on a dynamic rate. * @param input The
	 * target value
	 *
	 * @param rateLimit
	 *            The maximum units per second change (positive)
	 * @return The limited value
	 */
	public Translation2d calculate(Translation2d input, double accelLimit, double decelLimit) {
		double currentTime = Timer.getFPGATimestamp();
		double dt = MathUtil.clamp(currentTime - m_prevTime, 0.0, 0.1);

		// Check if we are accelerating or decelerating - change accel/decel based on if
		// above/below.
		double limit = input.getNorm() > m_prevVal.getNorm() ? accelLimit : decelLimit;

		// max acceptible change (in m/s)
		double maxDelta = Math.abs(limit) * dt;
		// actually change vector (in m/s/s)
		Translation2d delta = input.minus(m_prevVal);
		// actual change (in m/s/s)
		double deltaNorm = delta.getNorm();
		// if above the max change, then scale the vector such that it becomes correct.
		if (deltaNorm > maxDelta) {
			delta = delta.times(maxDelta / deltaNorm);
		}

		m_prevTime = currentTime;
		// change prev value by the acceptable change.z
		m_prevVal = m_prevVal.plus(delta);

		return m_prevVal;
	}

	public void reset(Translation2d value) {
		m_prevVal = value;
		m_prevTime = Timer.getFPGATimestamp();
	}
}
