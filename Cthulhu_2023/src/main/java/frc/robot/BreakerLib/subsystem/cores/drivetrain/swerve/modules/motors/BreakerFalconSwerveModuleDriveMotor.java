// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerCTREUtil;

/** Add your docs here. */
public class BreakerFalconSwerveModuleDriveMotor extends BreakerGenericSwerveModuleDriveMotor {
    private WPI_TalonFX motor;
    public BreakerFalconSwerveModuleDriveMotor(WPI_TalonFX motor, double driveGearRatio, boolean isMotorInverted, BreakerArbitraryFeedforwardProvider arbFF, BreakerSwerveMotorPIDConfig pidConfig) {
        this.motor = motor;
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        driveConfig.slot1.kP = pidConfig.kP;
        driveConfig.slot1.kI = pidConfig.kI;
        driveConfig.slot1.kD = pidConfig.kD;
        driveConfig.slot1.kF = pidConfig.kF;
        driveConfig.slot1.closedLoopPeakOutput = 1.0;
        driveConfig.peakOutputForward = 1.0;
        driveConfig.peakOutputReverse = -1.0;
        driveConfig.voltageCompSaturation = 12.0;
        driveConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 1.5);
        BreakerCTREUtil.checkError(motor.configAllSettings(driveConfig),
                " Failed to config swerve module drive motor ");
        motor.selectProfileSlot(1, 0);
        motor.setInverted(isMotorInverted);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.set(ControlMode.Velocity, 0.0);
    }

    @Override
    public void runSelfTest() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setTargetVelocity(double targetMetersPerSecond) {
         
        
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
