// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.ctre;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveAzimuthEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerGenericSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerSwerveAzimuthControler;
import frc.robot.BreakerLib.util.factory.BreakerProCANCoderFactory;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenixProUtil;

/** Add your docs here. */
public class BreakerProFalconSwerveModuleAngleMotor extends BreakerGenericSwerveModuleAngleMotor {
    private TalonFX motor;
    private BreakerSwerveAzimuthEncoder encoder;
    private Rotation2d targetAngle;
    private BreakerSwerveAzimuthControler azimuthControler;
    private final PositionDutyCycle positionRequest;
    public BreakerProFalconSwerveModuleAngleMotor(TalonFX motor, BreakerSwerveAzimuthEncoder encoder, double encoderAbsoluteAngleOffsetDegrees, double azimuthGearRatio, boolean isMotorInverted,  BreakerSwerveMotorPIDConfig pidConfig) {
        this.motor = motor;
        this.encoder = encoder;
        positionRequest = new PositionDutyCycle(0.0, true, 0.0, 0, false);

        encoder.config(false, encoderAbsoluteAngleOffsetDegrees);

        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        if (encoder.getBaseEncoderType() == CANcoder.class) {
            CANcoder cancoder = (CANcoder) encoder.getBaseEncoder();
            turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            turnConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
            turnConfig.Feedback.RotorToSensorRatio = azimuthGearRatio;
            turnConfig.Feedback.SensorToMechanismRatio = 1.0;
            turnConfig.Slot0.kP = pidConfig.kP;
            turnConfig.Slot0.kI = pidConfig.kI;
            turnConfig.Slot0.kD = pidConfig.kD;
            turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
            azimuthControler = new BreakerSwerveAzimuthControler((Rotation2d target) -> {motor.setControl(positionRequest.withPosition(target.getRotations()));});
        } else {
            azimuthControler = new BreakerSwerveAzimuthControler(motor, encoder, pidConfig);
        }
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        BreakerPhoenixProUtil.checkStatusCode(motor.getConfigurator().apply(turnConfig),
                " Failed to config swerve module drive motor ");
    
        motor.setInverted(isMotorInverted);
        azimuthControler.setTargetAngle(new Rotation2d());
        targetAngle = new Rotation2d();
        deviceName = "TalonFX_Swerve_Angle_Motor_(" + motor.getDeviceID() + ")";
    }

    @Override
    public void setTargetAngle(Rotation2d targetAngle) {
        azimuthControler.setTargetAngle(targetAngle);
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
        BreakerPhoenixProUtil.setBrakeMode(motor, isEnabled);
    }

    @Override
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        health = DeviceHealth.NOMINAL;
        Pair<DeviceHealth, String> motorPair = BreakerPhoenixProUtil.checkMotorFaultsAndConnection(motor);
        Pair<DeviceHealth, String> encoderPair = encoder.getFaultData();
        if (motorPair.getFirst() != DeviceHealth.NOMINAL || encoderPair.getFirst() != DeviceHealth.NOMINAL) {
            if (motorPair.getFirst() != DeviceHealth.NOMINAL) {
                faultStr += " ANGLE_MOTOR_FAULTS : " + motorPair.getSecond();
            }
            if (encoderPair.getFirst() != DeviceHealth.NOMINAL) {
                faultStr += " ENCODER_FAULTS : " + encoderPair.getSecond();
            }
        }
    }

}
