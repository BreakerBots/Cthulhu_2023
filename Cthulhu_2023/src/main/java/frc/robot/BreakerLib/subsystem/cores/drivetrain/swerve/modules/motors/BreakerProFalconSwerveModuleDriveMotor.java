// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors;

import com.ctre.phoenixpro.configs.AudioConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TorqueCurrentConfigs;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerCTREUtil;

/** Add your docs here. */
public class BreakerProFalconSwerveModuleDriveMotor extends BreakerGenericSwerveModuleDriveMotor {
    private TalonFX motor;
    private double driveGearRatio, wheelDiameter, targetVelocity;
    private BreakerArbitraryFeedforwardProvider arbFF;
    public BreakerProFalconSwerveModuleDriveMotor(TalonFX motor, double driveGearRatio, double wheelDiameter, boolean isMotorInverted, BreakerArbitraryFeedforwardProvider arbFF, BreakerSwerveMotorPIDConfig pidConfig) {
        this.motor = motor;
        this.driveGearRatio = driveGearRatio;
        this.wheelDiameter = wheelDiameter;
        this.arbFF = arbFF;
        targetVelocity = 0.0;
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfig.Slot1.kP = pidConfig.kP;
        driveConfig.Slot1.kI = pidConfig.kI;
        driveConfig.Slot1.kD = pidConfig.kD;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        driveConfig.peakOutputForward = 1.0;
        driveConfig.peakOutputReverse = -1.0;
        driveConfig.voltageCompSaturation = 12.0;
        driveConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 1.5);
        BreakerCTREUtil.checkError(motor.configAllSettings(driveConfig),
                " Failed to config swerve module drive motor ");
                private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 1, false);
        motor.getConfigurator().apply(driveConfig);
        motor.selectProfileSlot(1, 0);
        motor.setInverted(isMotorInverted);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.set(ControlMode.Velocity, 0.0);
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        Pair<DeviceHealth, String> pair = BreakerCTREUtil.checkMotorFaultsAndConnection(motor);
        health = pair.getFirst();
        if (health != DeviceHealth.NOMINAL) {
            faultStr = " DRIVE_MOTOR_FAULTS : " + pair.getSecond();
        }
    }

    @Override
    public void setTargetVelocity(double targetMetersPerSecond) {
        targetVelocity = targetMetersPerSecond;
        motor.set(TalonFXControlMode.Velocity, getMetersPerSecToNativeVelUnits(targetMetersPerSecond),
        DemandType.ArbitraryFeedForward, arbFF.getArbitraryFeedforwardValue(targetMetersPerSecond));
    }

    @Override
    public double getVelocity() {
        return Units.inchesToMeters(BreakerMath.ticksToInches(motor.getSelectedSensorVelocity() * 10,
        BreakerMath.getTicksPerInch(2048, driveGearRatio, wheelDiameter)));
    }

    @Override
    public double getDistance() {
        return Units.inchesToMeters(BreakerMath.ticksToInches(motor.getSelectedSensorPosition(),
            BreakerMath.getTicksPerInch(2048, driveGearRatio, wheelDiameter)));
    }

    @Override
    public void resetDistance() {
        motor.setSelectedSensorPosition(0);
    }

    @Override
    public void setBrakeMode(boolean isEnabled) {
        motor.setNeutralMode(isEnabled ? NeutralMode.Brake : NeutralMode.Coast);
        
    }

    @Override
    public double getTargetVelocity() {
        return targetVelocity;
    }

    private double getMetersPerSecToNativeVelUnits(double speedMetersPerSec) {
        return (speedMetersPerSec / 10) * Units.inchesToMeters(
                BreakerMath.getTicksPerInch(2048, driveGearRatio, wheelDiameter));
    }
}
