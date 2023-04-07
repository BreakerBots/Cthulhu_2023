// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerCTREUtil;

/** Add your docs here. */
public class BreakerFalconSwerveModuleDriveMotor extends BreakerGenericSwerveModuleDriveMotor {
    private WPI_TalonFX motor;
    private double driveGearRatio, wheelDiameter, targetVelocity;
    private BreakerArbitraryFeedforwardProvider arbFF;
    public BreakerFalconSwerveModuleDriveMotor(WPI_TalonFX motor, double driveGearRatio, double wheelDiameter, boolean isMotorInverted, BreakerArbitraryFeedforwardProvider arbFF, BreakerSwerveMotorPIDConfig pidConfig) {
        this.motor = motor;
        this.driveGearRatio = driveGearRatio;
        this.wheelDiameter = wheelDiameter;
        this.arbFF = arbFF;
        targetVelocity = 0.0;
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
