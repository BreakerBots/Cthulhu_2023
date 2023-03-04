// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;

/** Add your docs here. */
public abstract class BreakerGenericSwerveModuleAngleMotor implements BreakerGenericDevice {
    protected BreakerSwerveModuleAngleMotorPIDConfig pidConfig;
    protected boolean isInverted;
    public BreakerGenericSwerveModuleAngleMotor(BreakerSwerveModuleAngleMotorPIDConfig pidConfig, boolean isInverted) {
        this.pidConfig = pidConfig;
        this.isInverted = isInverted;
    }
    public abstract void setTargetAngle(Rotation2d targetAngle);
    public abstract double getAblsoluteAngle();
    public abstract double getRelativeAngle(); 
    public abstract void setBrakeMode(boolean isEnabled);
    public abstract void getBrakeMode(boolean isEnabled);


    public static class BreakerSwerveModuleAngleMotorPIDConfig {
        public final double kP, kI, kD, kF;
        public BreakerSwerveModuleAngleMotorPIDConfig(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }
    }

}
