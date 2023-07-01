// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.offseasionbot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.driverstation.dashboard.BreakerDashboard;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveBase;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerGenericSwerveModule;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModuleBuilder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModuleBuilder.BreakerSwerveModuleConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveAzimuthEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveCANcoder;
import static frc.robot.subsystems.offseasionbot.OffseasionBotConstants.DriveConstants.*;
import static frc.robot.subsystems.offseasionbot.OffseasionBotConstants.MiscConstants.CANIVORE_1;;

/** Add your docs here. */
public class OffseasionBotDrive extends BreakerSwerveDriveBase {

    private static TalonFX driveFL = new TalonFX(FL_DRIVE_ID, CANIVORE_1);
    private static TalonFX turnFL = new TalonFX(FL_TURN_ID, CANIVORE_1);
    private static BreakerSwerveAzimuthEncoder encoderFL = new BreakerSwerveCANcoder(new CANcoder(FL_ENCODER_ID, CANIVORE_1));

    private static TalonFX driveFR = new TalonFX(FR_DRIVE_ID, CANIVORE_1);
    private static TalonFX turnFR = new TalonFX(FR_TURN_ID, CANIVORE_1);
    private static BreakerSwerveAzimuthEncoder encoderFR =  new BreakerSwerveCANcoder(new CANcoder(FR_ENCODER_ID, CANIVORE_1));

    private static TalonFX driveBL = new TalonFX(BL_DRIVE_ID, CANIVORE_1);
    private static TalonFX turnBL = new TalonFX(BL_TURN_ID, CANIVORE_1);
    private static BreakerSwerveAzimuthEncoder encoderBL =  new BreakerSwerveCANcoder(new CANcoder(BL_ENCODER_ID, CANIVORE_1));

    private static TalonFX driveBR = new TalonFX(BR_DRIVE_ID, CANIVORE_1);
    private static TalonFX turnBR = new TalonFX(BR_TURN_ID, CANIVORE_1);
    private static BreakerSwerveAzimuthEncoder encoderBR =  new BreakerSwerveCANcoder(new CANcoder(BR_ENCODER_ID, CANIVORE_1));

    private static BreakerSwerveModule frontLeftModule = BreakerSwerveModuleBuilder.getInstance(MODULE_CONFIG)
        .withFalconAngleMotor(turnFL, encoderFL, FL_ENCODER_OFFSET, true)
        .withFalconDriveMotor(driveFL, true)
        .createSwerveModule(FL_TRANSLATION);

    private static BreakerSwerveModule frontRightModule = BreakerSwerveModuleBuilder.getInstance(MODULE_CONFIG)
        .withFalconAngleMotor(turnFR, encoderFR, FR_ENCODER_OFFSET, true)
        .withFalconDriveMotor(driveFR, false)
        .createSwerveModule(FR_TRANSLATION);

    private static BreakerSwerveModule backLeftModule = BreakerSwerveModuleBuilder.getInstance(MODULE_CONFIG)
        .withFalconAngleMotor(turnBL, encoderBL, BL_ENCODER_OFFSET, true)
        .withFalconDriveMotor(driveBL, true)
        .createSwerveModule(BL_TRANSLATION);

    private static BreakerSwerveModule backRightModule = BreakerSwerveModuleBuilder.getInstance(MODULE_CONFIG)
        .withFalconAngleMotor(turnBR, encoderBR, BR_ENCODER_OFFSET, true)
        .withFalconDriveMotor(driveBR, false)
        .createSwerveModule(BR_TRANSLATION);

    private static Field2d field = new Field2d();
    

    public OffseasionBotDrive(BreakerPigeon2 pigeon) {
        super(DRIVE_BASE_CONFIG, pigeon, frontLeftModule, frontRightModule, backLeftModule, backRightModule);

        BreakerDashboard.getMainTab().add(field);
        
        frontLeftModule.setDeviceName(" FL_Module ");
        frontRightModule.setDeviceName(" FR_Module ");
        backLeftModule.setDeviceName(" BL_Module ");
        backRightModule.setDeviceName(" BR_Module ");

        BreakerDashboard.getDiagnosticsTab().add("FL Module", frontLeftModule);
        BreakerDashboard.getDiagnosticsTab().add("FR Module", frontRightModule);
        BreakerDashboard.getDiagnosticsTab().add("BL Module", backLeftModule);
        BreakerDashboard.getDiagnosticsTab().add("BR Module", backRightModule);
    }

    @Override
    public void periodic() {
        field.setRobotPose(getOdometryPoseMeters());
    }
}
