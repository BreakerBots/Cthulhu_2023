// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;

/** Add your docs here. */
public abstract class BreakerGenericSwerveModuleAngleMotor implements BreakerGenericDevice {
    public abstract void setTargetAngle(Rotation2d targetAngle);
    public abstract double getAblsoluteAngle();
    public abstract double getRelativeAngle(); 
    public abstract void setBrakeMode(boolean isEnabled);
    public abstract boolean getBrakeMode();
    public abstract Rotation2d getTargetAngle();

}
