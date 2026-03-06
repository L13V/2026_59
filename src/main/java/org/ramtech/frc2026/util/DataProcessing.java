package org.ramtech.frc2026.util;

public class DataProcessing {
	/**
	 * Sanitizes an input value by handling NaN and clamping to a range.
	 *
	 * <p>
	 * If the input is NaN, it attempts to use the last valid value. If the last
	 * value is also NaN, it defaults to the minimum value. Finally, the result is
	 * clamped between the specified minimum and maximum values.
	 *
	 * @param last
	 *            The last valid value, used as a fallback if input is NaN.
	 * @param min
	 *            The minimum allowed value.
	 * @param max
	 *            The maximum allowed value.
	 * @param input
	 *            The current input value to sanitize.
	 * @return The sanitized value.
	 */
	public static double sanitize(double last, double min, double max, double input) {
		double sanitized;

		if (Double.isNaN(input)) {
			if (Double.isNaN(last)) {
				sanitized = min;
			} else {
				sanitized = last;
			}
		} else {
			sanitized = input;
		}

		if (sanitized < min)
			sanitized = min;
		if (sanitized > max)
			sanitized = max;
		return sanitized;
	}

	/**
	 * Smooths a raw input value using a weighted average with the previous smoothed
	 * value.
	 *
	 * @param samplecount
	 *            The weight of the history. Higher values result in more smoothing.
	 *            Effectively acts as the denominator in the weighted average.
	 * @param last
	 *            The previous smoothed value.
	 * @param raw
	 *            The current raw input value.
	 * @return The new smoothed value.
	 */
	public static double rawToSmooth(int samplecount, double last, double raw) {
		double smoothed;
		smoothed = (raw + ((samplecount - 1) * last)) / samplecount;
		return smoothed;
	}
}
