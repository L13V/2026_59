package org.ramtech.frc2026.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class DynamicRateLimiter {
	private double m_prevVal;
	private double m_prevTime;

	/**
	 * @param initialValue
	 *            The value to start at (usually 0)
	 */
	public DynamicRateLimiter(double initialValue) {
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
	public double calculate(double input, double accelLimit, double decelLimit) {
		double currentTime = Timer.getFPGATimestamp();
		double dt = currentTime - m_prevTime;
		m_prevTime = currentTime;

		double limit;

		// Check if we are accelerating or decelerating
		// Math.abs(input) > Math.abs(m_prevVal) means we are increasing speed
		if (Math.abs(input) > Math.abs(m_prevVal)) {
			limit = accelLimit;
		} else {
			limit = decelLimit;
		}

		m_prevVal += MathUtil.clamp(input - m_prevVal, -limit * dt, limit * dt);

		return m_prevVal;
	}

	public void reset(double value) {
		m_prevVal = value;
		m_prevTime = Timer.getFPGATimestamp();
	}
}
