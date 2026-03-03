package org.ramtech.frc2026.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

public class PhoenixUtil {
	/** Attempts to run the command until no error is produced. */
	public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
		for (int i = 0; i < maxAttempts; i++) {
			var error = command.get();
			if (error.isOK())
				break;
		}
	}

	public static boolean tryUntilOkWithStatus(int maxAttempts, Supplier<StatusCode> command) {
		for (int i = 0; i < maxAttempts; i++) {
			var error = command.get();
			if (error.isOK())
				return error.isOK();
		}
		return false;
	}
}
