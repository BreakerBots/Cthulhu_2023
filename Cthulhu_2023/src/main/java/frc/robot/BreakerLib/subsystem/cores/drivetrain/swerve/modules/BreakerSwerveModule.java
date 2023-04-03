// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.BreakerGenericSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.BreakerGenericSwerveModuleDriveMotor;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Add your docs here. */
public class BreakerSwerveModule implements BreakerGenericSwerveModule {
    private BreakerGenericSwerveModuleDriveMotor driveMotor;
    private BreakerGenericSwerveModuleAngleMotor angleMotor;
    public BreakerSwerveModule(BreakerGenericSwerveModuleDriveMotor driveMotor, BreakerGenericSwerveModuleAngleMotor angleMotor) {
        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;
    }

    @Override
    public void runSelfTest() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public DeviceHealth getHealth() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public String getFaults() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public String getDeviceName() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public boolean hasFault() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void setDeviceName(String newName) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setModuleTarget(Rotation2d targetAngle, double targetVelocityMetersPerSecond) {
        angleMotor.setTargetAngle(targetAngle);
        driveMotor.setTargetVelocity(targetVelocityMetersPerSecond);
    }

    @Override
    public double getModuleAbsoluteAngle() {
        return angleMotor.getRelativeAngle();
    }

    @Override
    public double getModuleRelativeAngle() {
        return angleMotor.getRelativeAngle();
    }

    @Override
    public double getModuleVelMetersPerSec() {
        return driveMotor.getVelocity();
    }

    @Override
    public double getMetersPerSecToNativeVelUnits(double speedMetersPerSec) {
        return 0.0;
    }

    @Override
    public SwerveModuleState getModuleTargetState() {
        return new SwerveModuleState(driveMotor.getVelocity(), angleMotor.getTargetAngle());
    }

    @Override
    public void setDriveMotorBrakeMode(boolean isEnabled) {
        driveMotor.setBrakeMode(isEnabled);
    }

    @Override
    public void setTurnMotorBrakeMode(boolean isEnabled) {
        angleMotor.setBrakeMode(isEnabled);
        
    }

    @Override
    public void setModuleBrakeMode(boolean isEnabled) {
        setDriveMotorBrakeMode(isEnabled);
        setTurnMotorBrakeMode(isEnabled);
    }

    @Override
    public double getModuleDriveDistanceMeters() {
        return driveMotor.getDistance();
    }

    @Override
    public void resetModuleDriveEncoderPosition() {
        driveMotor.resetDistance();
        
    }

    @Override
    public DeviceHealth[] getModuleHealths() {
        return null;
    }

    public static class BreakerSwerveMotorPIDConfig {
        public final double kP, kI, kD, kF;
        public BreakerSwerveMotorPIDConfig(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }
    }
}
