// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveAzimuthEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerGenericSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.ctre.BreakerFalconSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.ctre.BreakerProFalconSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.rev.BreakerNeoSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.BreakerGenericSwerveModuleDriveMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre.BreakerFalconSwerveModuleDriveMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre.BreakerProFalconSwerveModuleDriveMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.rev.BreakerNeoSwerveModuleDriveMotor;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;

/** Add your docs here. */
public class BreakerSwerveModuleBuilder {
    private BreakerSwerveModuleConfig config;
    private BreakerGenericSwerveModuleAngleMotor angleMotor;
    private BreakerGenericSwerveModuleDriveMotor driveMotor;
    private BreakerSwerveModuleBuilder(BreakerSwerveModuleConfig config) {
        this.config = config;
    }

    public static BreakerSwerveModuleBuilder getInstance(BreakerSwerveModuleConfig config) {
        return new BreakerSwerveModuleBuilder(config);
    }

    public BreakerSwerveModuleBuilder withFalconAngleMotor(WPI_TalonFX motor, BreakerSwerveAzimuthEncoder encoder, double encoderAbsoluteAngleOffsetDegrees, boolean isMotorInverted) {
        angleMotor = new BreakerFalconSwerveModuleAngleMotor(motor, encoder, encoderAbsoluteAngleOffsetDegrees, isMotorInverted, config.getAnglePIDConfig());
        return this;
    }

    public BreakerSwerveModuleBuilder withFalconDriveMotor(WPI_TalonFX motor, boolean isMotorInverted) {
        driveMotor = new BreakerFalconSwerveModuleDriveMotor(motor, config.getDriveGearRatio(), config.getWheelDiameter(), isMotorInverted, config.getDriveArbFF(), config.getDrivePIDConfig());
        return this;
    }

    public BreakerSwerveModuleBuilder withProFalconAngleMotor(TalonFX motor, BreakerSwerveAzimuthEncoder encoder, double encoderAbsoluteAngleOffsetDegrees, boolean isMotorInverted) {
        angleMotor = new BreakerProFalconSwerveModuleAngleMotor(motor, encoder, config.getAzimuthGearRatio(), encoderAbsoluteAngleOffsetDegrees, isMotorInverted, config.getAnglePIDConfig());
        return this;
    }

    public BreakerSwerveModuleBuilder withProFalconDriveMotor(TalonFX motor, boolean isMotorInverted) {
        driveMotor = new BreakerProFalconSwerveModuleDriveMotor(motor, config.getDriveGearRatio(), config.getWheelDiameter(), isMotorInverted, config.getDriveArbFF(), config.getDrivePIDConfig());
        return this;
    }

    public BreakerSwerveModuleBuilder withNEOAngleMotor(CANSparkMax motor, BreakerSwerveAzimuthEncoder encoder, double encoderAbsoluteAngleOffsetDegrees, boolean isMotorInverted) {
        angleMotor = new BreakerNeoSwerveModuleAngleMotor(motor, encoder, encoderAbsoluteAngleOffsetDegrees, isMotorInverted, config.getAnglePIDConfig());
        return this;
    }

    public BreakerSwerveModuleBuilder withNEODriveMotor(CANSparkMax motor, boolean isMotorInverted) {
        driveMotor = new BreakerNeoSwerveModuleDriveMotor(motor, config.getDriveGearRatio(), config.getWheelDiameter(), isMotorInverted, config.getDriveArbFF(), config.getDrivePIDConfig());
        return this;
    }

    public BreakerSwerveModule createSwerveModule(Translation2d wheelPositionRelativeToRobot) {
        return new BreakerSwerveModule(driveMotor, angleMotor, wheelPositionRelativeToRobot);
    }

    public static class BreakerSwerveModuleConfig {
        private double driveGearRatio, azimuthGearRatio, wheelDiameter;
        private BreakerSwerveMotorPIDConfig anglePIDConfig, drivePIDConfig;
        private BreakerArbitraryFeedforwardProvider driveArbFF;
        public BreakerSwerveModuleConfig(double driveGearRatio, double azimuthGearRatio, double wheelDiameter, BreakerSwerveMotorPIDConfig anglePIDConfig, BreakerSwerveMotorPIDConfig drivePIDConfig, BreakerArbitraryFeedforwardProvider driveArbFF) {
            this.driveGearRatio = driveGearRatio;
            this.azimuthGearRatio = azimuthGearRatio;
            this.wheelDiameter = wheelDiameter;
            this.drivePIDConfig = drivePIDConfig;
            this.anglePIDConfig = anglePIDConfig;
            this.driveArbFF = driveArbFF;
        }

        public BreakerSwerveMotorPIDConfig getAnglePIDConfig() {
            return anglePIDConfig;
        }

        public BreakerSwerveMotorPIDConfig getDrivePIDConfig() {
            return drivePIDConfig;
        }

        public BreakerArbitraryFeedforwardProvider getDriveArbFF() {
            return driveArbFF;
        }

        public double getDriveGearRatio() {
            return driveGearRatio;
        }

        public double getAzimuthGearRatio() {
            return azimuthGearRatio;
        }

        public double getWheelDiameter() {
            return wheelDiameter;
        }
    }
}
