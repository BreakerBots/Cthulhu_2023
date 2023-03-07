// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.modulebuilder;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;

import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveModulePIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.BreakerFalconSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.BreakerGenericSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.BreakerGenericSwerveModuleDriveMotor;

/** Add your docs here. */
public class BreakerSwerveModuleBuilder {
    private BreakerGenericSwerveModuleDriveMotor driveMotor;
    private BreakerGenericSwerveModuleAngleMotor angleMotor;
    public void withFalconAngleMotor(WPI_TalonFX motor, WPI_CANCoder encoder, double encoderOffsetAngle, boolean isAngleMotorInverted)  {
        angleMotor = new BreakerFalconSwerveModuleAngleMotor(motor, encoder, 0, isAngleMotorInverted, null);
    }

    public void withFalconAngleMotor(WPI_TalonFX motor, WPI_CANCoder encoder, BreakerSwerveModulePIDConfig pidConfig)  {

    }

    public void withFalconDriveMotor(WPI_TalonFX motor, WPI_CANCoder encoder)  {

    }

    public void withFalconDriveMotor(WPI_TalonFX motor, WPI_CANCoder encoder, BreakerSwerveModulePIDConfig pidConfig)  {
        
    }

    public void withNEOAngleMotor(CANSparkMax motor, WPI_CANCoder encoder)  {

    }

    public void withNEOAngleMotor(CANSparkMax motor, WPI_CANCoder encoder, BreakerSwerveModulePIDConfig pidConfig)  {

    }

    public void withNEODriveMotor(CANSparkMax motor, WPI_CANCoder encoder)  {

    }

    public void withNEODriveMotor(CANSparkMax motor, WPI_CANCoder encoder, BreakerSwerveModulePIDConfig pidConfig)  {
        
    }


}
