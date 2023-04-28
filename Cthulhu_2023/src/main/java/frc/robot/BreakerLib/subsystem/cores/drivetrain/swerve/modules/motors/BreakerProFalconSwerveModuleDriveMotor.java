// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors;

import com.ctre.phoenixpro.configs.AudioConfigs;
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TorqueCurrentConfigs;
import com.ctre.phoenixpro.controls.NeutralOut;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerCTREUtil;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenixProUtil;

/** Add your docs here. */
public class BreakerProFalconSwerveModuleDriveMotor extends BreakerGenericSwerveModuleDriveMotor {
    private TalonFX motor;
    private double driveGearRatio, wheelDiameter, targetVelocity;
    private final VelocityTorqueCurrentFOC torqueVelocity;
    private final double wheelCircumfrenceMeters;
    private BreakerArbitraryFeedforwardProvider arbFF;
    public BreakerProFalconSwerveModuleDriveMotor(TalonFX motor, double driveGearRatio, double wheelDiameter, boolean isMotorInverted, BreakerArbitraryFeedforwardProvider arbFF, BreakerSwerveMotorPIDConfig pidConfig) {
        this.motor = motor;
        this.driveGearRatio = driveGearRatio;
        this.wheelDiameter = wheelDiameter;
        this.arbFF = arbFF;
        wheelCircumfrenceMeters = Units.inchesToMeters(wheelDiameter*Math.PI);
        targetVelocity = 0.0;
        torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 1, false);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfig.Feedback.SensorToMechanismRatio = driveGearRatio;
        driveConfig.Slot1.kP = pidConfig.kP;
        driveConfig.Slot1.kI = pidConfig.kI;
        driveConfig.Slot1.kD = pidConfig.kD;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        BreakerPhoenixProUtil.checkStatusCode(motor.getConfigurator().apply(driveConfig),
                " Failed to config swerve module drive motor ");
    
        motor.setInverted(isMotorInverted);
        motor.setControl(torqueVelocity);
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        Pair<DeviceHealth, String> pair = BreakerPhoenixProUtil.checkMotorFaultsAndConnection(motor);
        health = pair.getFirst();
        if (health != DeviceHealth.NOMINAL) {
            faultStr = " DRIVE_MOTOR_FAULTS : " + pair.getSecond();
        }
    }

    @Override
    public void setTargetVelocity(double targetMetersPerSecond) {
        targetVelocity = targetMetersPerSecond;
        motor.setControl(
            torqueVelocity.withVelocity(targetMetersPerSecond / wheelCircumfrenceMeters)
            .withFeedForward(arbFF.getArbitraryFeedforwardValue(targetMetersPerSecond))
            );
    }

    @Override
    public double getVelocity() {
        return motor.getVelocity().getValue() * wheelCircumfrenceMeters;
    }

    @Override
    public double getDistance() {
        return motor.getPosition().getValue() * wheelCircumfrenceMeters;
    }

    @Override
    public void resetDistance() {
        BreakerPhoenixProUtil.checkStatusCode(motor.setRotorPosition(0),
                " Failed to reset swerve module rive motor position ");
        ;
    }

    @Override
    public void setBrakeMode(boolean isEnabled) {
        BreakerPhoenixProUtil.setBrakeMode(motor, isEnabled);
    }

    @Override
    public double getTargetVelocity() {
        return targetVelocity;
    }
}
