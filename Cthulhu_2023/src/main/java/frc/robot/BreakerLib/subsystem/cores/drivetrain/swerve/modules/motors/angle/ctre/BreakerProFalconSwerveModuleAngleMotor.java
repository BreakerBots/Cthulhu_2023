// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.ctre;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
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
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerGenericSwerveModuleAngleMotor;
import frc.robot.BreakerLib.util.factory.BreakerProCANCoderFactory;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenixProUtil;

/** Add your docs here. */
public class BreakerProFalconSwerveModuleAngleMotor extends BreakerGenericSwerveModuleAngleMotor {
    private TalonFX motor;
    private CANcoder encoder;
    private Rotation2d targetAngle;
    private final PositionTorqueCurrentFOC torquePosition;
    public BreakerProFalconSwerveModuleAngleMotor(TalonFX motor, CANcoder encoder, double encoderAbsoluteAngleOffsetRotations, double azimuthGearRatio, boolean isMotorInverted,  BreakerSwerveMotorPIDConfig pidConfig) {
        this.motor = motor;
        this.encoder = encoder;
        torquePosition  = new PositionTorqueCurrentFOC(0, 0, 0, false);

        BreakerProCANCoderFactory.configExistingCANCoder(encoder, AbsoluteSensorRangeValue.Signed_PlusMinusHalf, encoderAbsoluteAngleOffsetRotations, SensorDirectionValue.CounterClockwise_Positive);

        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        turnConfig.Feedback.RotorToSensorRatio = azimuthGearRatio;
        turnConfig.Feedback.SensorToMechanismRatio = 1.0;
        turnConfig.Slot0.kP = pidConfig.kP;
        turnConfig.Slot0.kI = pidConfig.kI;
        turnConfig.Slot0.kD = pidConfig.kD;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        BreakerPhoenixProUtil.checkStatusCode(motor.getConfigurator().apply(turnConfig),
                " Failed to config swerve module drive motor ");
    
        motor.setInverted(isMotorInverted);
        motor.setControl(torquePosition);
        targetAngle = new Rotation2d();
        deviceName = "TalonFX_Swerve_Angle_Motor_(" + motor.getDeviceID() + ")";
    }

    @Override
    public void setTargetAngle(Rotation2d targetAngle) {
        motor.setControl(torquePosition.withPosition(targetAngle.getRotations()));
        this.targetAngle = targetAngle;
        
    }

    @Override
    public double getAblsoluteAngle() {
        return encoder.getAbsolutePosition().getValue() * 360.0;
    }

    @Override
    public double getRelativeAngle() {
        return encoder.getPosition().getValue() * 360.0;
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
        Pair<DeviceHealth, String> encoderPair = BreakerPhoenixProUtil.checkCANcoderFaultsAndConnection(encoder);
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
