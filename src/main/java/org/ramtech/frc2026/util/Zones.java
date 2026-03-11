package org.ramtech.frc2026.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;

public class Zones {

	public static enum zoneType {
		scoring, passingcloseleft, passingcloseright, passingfarleft, passingfarright, blockShooting, hoodUnsafe, nothing
	};

	private static final double halfTurretLength = Units.inchesToMeters(30) / 2;
	private static final double halfTurretWidth = Units.inchesToMeters(30) / 2;

	// 2. Define a clean inner record to hold zone coordinates
	public record Zone(double x1, double y1, double x2, double y2, String name, zoneType type) {
		public boolean containsPoint(Translation2d point) {
			return point.getX() >= x1 && point.getX() <= x2 && point.getY() >= y1 && point.getY() <= y2;
		}
	}

	// 3. List to hold all the areas where the hood needs to be down
	private final List<Zone> zones = new ArrayList<>();

	/**
	 * Adds a new unsafe rectangular area to the checker.
	 */
	public void addZone(double x1, double y1, double x2, double y2, String name, zoneType type) {
		zones.add(new Zone(x1, y1, x2, y2, name, type));
	}

	public Zones() {
		// Our alliance zone
		// Our Left Trench
		// Our Right Trench

		// Opposing alliance zone
		// Their Left Trench
		// Their Right Trench

		// Center

		zones.add(new Zone(4.028694, 6.80339, 4.028694 + 1.1938, 8.069326, "Our Left Trench", zoneType.hoodUnsafe));
		zones.add(new Zone(4.028694, 0, 4.028694 + 1.1938, 1.265936, "Our Right Trench", zoneType.hoodUnsafe));

		zones.add(new Zone(11.318496, 6.80339, 11.318496 + 1.1938, 8.069326, "Opposing Right Trench",
				zoneType.hoodUnsafe));
		zones.add(new Zone(11.318496, 0, 11.318496 + 1.1938, 1.265936, "Opposing Left Trench", zoneType.hoodUnsafe));

		zones.add(new Zone(0, 0, 5.222494, 8.069326, "Our Alliance", zoneType.scoring));
		zones.add(new Zone(5.222494, (8.069326 / 2) + 0.1, 11.318496, 8.069326, "Left Center",
				zoneType.passingcloseleft));
		zones.add(new Zone(5.222494, 0, 11.318496, (8.069326 / 2) - 0.1, "Right Center", zoneType.passingcloseright));

		// zones.add(new Zone(11.318496, 0, 16.540988, 8.069326, "Opposing Alliance",
		// zoneType.passingcloseleft));
		zones.add(new Zone(11.318496, (8.069326 / 2) + 0.1, 16.540988, 8.069326, "Opposing Alliance Left",
				zoneType.passingfarleft));
		zones.add(new Zone(11.318496, 0, 16.540988, (8.069326 / 2) - 0.1, "Opposing Alliance Right",
				zoneType.passingfarright));

	}

	/**
	 * Calculates the 8 perimeter points of the turret based on its pose.
	 */
	private Translation2d[] getTurretPoints(Pose2d turretPose) {
		Translation2d[] localPoints = new Translation2d[]{new Translation2d(halfTurretLength, halfTurretWidth), // Front
																												// Left
				new Translation2d(halfTurretLength, -halfTurretWidth), // Front Right
				new Translation2d(-halfTurretLength, halfTurretWidth), // Back Left
				new Translation2d(-halfTurretLength, -halfTurretWidth), // Back Right
				new Translation2d(halfTurretLength, 0), // Front Center
				new Translation2d(-halfTurretLength, 0), // Back Center
				new Translation2d(0, halfTurretWidth), // Left Center
				new Translation2d(0, -halfTurretWidth) // Right Center
		};

		Translation2d[] fieldPoints = new Translation2d[8];
		for (int i = 0; i < localPoints.length; i++) {
			fieldPoints[i] = localPoints[i].rotateBy(turretPose.getRotation()).plus(turretPose.getTranslation());
		}

		return fieldPoints;
	}

	/**
	 * Checks if ANY part of the robot's turret are inside ANY unsafe zone.
	 */
	public boolean isTurretUnsafe(Pose2d turretPose) {
		Translation2d[] turretPoints = getTurretPoints(turretPose);

		for (Zone zone : zones) {
			for (Translation2d point : turretPoints) {
				if (zone.containsPoint(point) && zone.type() == zoneType.hoodUnsafe) {
					return true; // Lower hood
				}
			}
		}
		return false; // allow hood raise
	}

	public Zone isTurretUnsafeZone(Pose2d turretPose) {
		Translation2d[] turretPoints = getTurretPoints(turretPose);

		for (Zone zone : zones) {
			for (Translation2d point : turretPoints) {
				if (zone.type() == zoneType.hoodUnsafe && zone.containsPoint(point)) {
					return zone; // Lower hood
				}
			}
		}
		return null; // allow hood raise
	}

	/**
	 * checks if the pose is inside a zone and returns it
	 */
	public Zone getZoneFromPose(Pose2d pose) {
		// Prioritize unsafe zones with lengthy check
		var turretzone = isTurretUnsafeZone(pose);
		if (turretzone != null) {
			return turretzone;
		}
		// check for other zones
		for (Zone zone : zones) {
			if (zone.containsPoint(pose.getTranslation())) {
				return zone;
			}
		}
		return null;
	}
}
