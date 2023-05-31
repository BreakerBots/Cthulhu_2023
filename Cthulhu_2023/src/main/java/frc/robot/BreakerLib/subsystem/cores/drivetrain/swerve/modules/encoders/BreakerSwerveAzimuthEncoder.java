// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

// Preferably this becomes a general duty cycle encoder interface.

/** Add your docs here. */
public interface BreakerSwerveAzimuthEncoder {

    // Unused, probably should be removed. More fitting for a quad encoder.
    public abstract double getRelative();

    /** @return Absolute anglular position in degrees [-180, 180]. */
    public abstract double getAbsolute();

    /**
     * Configures the encoder.
     * 
     * @param clockwisePositive Sets whether clockwise rotation is positive or
     *                          negative.
     * @param offset            Angle offset in degrees.
     */
    public abstract void config(boolean clockwisePositive, double offset);

    public abstract Pair<DeviceHealth, String> getFaultData();

    public abstract Class<?> getBaseEncoderType();

    public abstract Object getBaseEncoder();
}