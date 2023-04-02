// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveModulePIDConfig;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Add your docs here. */
public class BreakerFalconSwerveModuleDriveMotor implements BreakerGenericSwerveModuleDriveMotor {

    public BreakerFalconSwerveModuleDriveMotor(WPI_TalonFX motor, double driveGearRatio, boolean isMotorInverted, BreakerArbitraryFeedforwardProvider arbFF, BreakerSwerveModulePIDConfig pidConfig) {

    }

    @Override
    public DevicePowerMode managePower(BreakerPowerManagementConfig managementConfig, double... managementPerameters) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void overrideAutomaticPowerManagement(DevicePowerMode manualPowerMode) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void returnToAutomaticPowerManagement() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isUnderAutomaticControl() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public DevicePowerMode getPowerMode() {
        // TODO Auto-generated method stub
        return null;
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
    public void setTargetVelocity(double targetMetersPerSecond) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getVelocity() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getDistance() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void resetDistance() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setBrakeMode(boolean isEnabled) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getTargetVelocity() {
        // TODO Auto-generated method stub
        return 0;
    }}
