// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * A path of positional waypoints. Rotation must be affected separately with a
 * supplier
 */
public class BreakerWaypointPath {
    private final double maxVelocity;
    private final boolean stopAtEnd;
    private final Translation2d[] waypoints;

    public BreakerWaypointPath(double maxVelocity, boolean stopAtEnd, Translation2d... waypoints) {
        this.maxVelocity = maxVelocity;
        this.stopAtEnd = stopAtEnd;
        this.waypoints = waypoints;
    }

    /** @return Total distance between all waypoints in meters. */
    public double getTotalPathDistance() {
        double dist = 0;
        for (int i = 1; i < waypoints.length; i++) {
            dist += waypoints[i - 1].getDistance(waypoints[i]);
        }
        return dist;
    }

    /** @return Max vel and accel constraints from trapezoid profile. */
    public double getMaxVelocity() {
        return cons;
    }

    /** @return Array of 2d waypoints. */
    public Translation2d[] getWaypoints() {
        return waypoints;
    }

    /**
     * @return New waypoint path with all points from both paths and mean trapezoid profile constraints.
     * 
     * @param other Other waypoint path.
     */
    public BreakerWaypointPath concatenate(BreakerWaypointPath other) {
        Translation2d[] newWaypoints = new Translation2d[getWaypoints().length + other.getWaypoints().length];
        for (int i = 0; i < newWaypoints.length; i++) {
            if (i < waypoints.length) {
                newWaypoints[i] = waypoints[i];
            } else {
                newWaypoints[i] = other.getWaypoints()[i - (newWaypoints.length - 1)];
            }
        }
        return new BreakerWaypointPath(
                new TrapezoidProfile.Constraints(
                        (constraints.maxVelocity + other.constraints.maxVelocity) / 2.0,
                        (constraints.maxAcceleration + other.constraints.maxAcceleration) / 2.0),
                newWaypoints);
    }
}
