// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.rev;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveAzimuthEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerGenericSwerveModuleAngleMotor;
//import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveModulePIDConfig;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerCTREUtil;
import frc.robot.BreakerLib.util.vendorutil.BreakerREVUtil;

/** Add your docs here. */
public class BreakerNeoSwerveModuleAngleMotor extends BreakerGenericSwerveModuleAngleMotor {
    private CANSparkMax motor;
    private BreakerSwerveAzimuthEncoder encoder;
    private Rotation2d targetAngle;
    private PIDController pid;
    public BreakerNeoSwerveModuleAngleMotor(CANSparkMax motor, BreakerSwerveAzimuthEncoder encoder, double encoderAbsoluteAngleOffsetDegrees, boolean isMotorInverted,  BreakerSwerveMotorPIDConfig pidConfig) {
        this.motor = motor;
        this.encoder = encoder;
        
        deviceName = "NEO_Swerve_Angle_Motor_(" + motor.getDeviceId() + ")";
        encoder.config(false, encoderAbsoluteAngleOffsetDegrees);

        BreakerREVUtil.checkError(motor.enableVoltageCompensation(12.0), "Failed to config " + deviceName + " voltage compensation");
        BreakerREVUtil.checkError(motor.setSmartCurrentLimit(80),  "Failed to config " + deviceName + " smart current limit");
        motor.setInverted(isMotorInverted);
        pid = new PIDController(pidConfig.kP, pidConfig.kI, pidConfig.kD);
        pid.enableContinuousInput(-180, 180);
        targetAngle = new Rotation2d();
    }

    @Override
    public void setTargetAngle(Rotation2d targetAngle) {
        motor.set(pid.calculate(targetAngle.getDegrees(), encoder.getAbsolute()));
        this.targetAngle = targetAngle;
        
    }

    @Override
    public double getAbsoluteAngle() {
        return encoder.getAbsolute();
    }

    @Override
    public double getRelativeAngle() {
        return encoder.getRelative();
    }
    @Override
    public void setBrakeMode(boolean isEnabled) {
        motor.setIdleMode(isEnabled ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        health = DeviceHealth.NOMINAL;
        Pair<DeviceHealth, String> motorPair = BreakerREVUtil.getSparkMaxHealthAndFaults(motor.getFaults());
        Pair<DeviceHealth, String> encoderPair = encoder.getFaultData();
        if (encoderPair.getFirst() != DeviceHealth.NOMINAL || encoderPair.getFirst() != DeviceHealth.INOPERABLE) {
            health = DeviceHealth.INOPERABLE;
            if (motorPair.getFirst() != DeviceHealth.NOMINAL) {
                faultStr += " ANGLE_MOTOR_FAULTS : " + motorPair.getSecond();
            }
            if (encoderPair.getFirst() != DeviceHealth.NOMINAL) {
                faultStr += " ENCODER_FAULTS : " + encoderPair.getSecond();
            }
        }
    }

}