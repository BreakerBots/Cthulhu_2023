// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveModulePIDConfig;
//import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveModulePIDConfig;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerUnits;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerCTREUtil;

/** Add your docs here. */
public class BreakerFalconSwerveModuleAngleMotor extends BreakerGenericSwerveModuleAngleMotor {
    private WPI_TalonFX motor;
    private WPI_CANCoder encoder;
    private BreakerSwerveModulePIDConfig pidConfig;
    private Rotation2d targetAngle;
    private String deviceName;
    public BreakerFalconSwerveModuleAngleMotor(WPI_TalonFX motor, WPI_CANCoder encoder, double encoderAbsoluteAngleOffsetDegrees, boolean isMotorInverted,  BreakerSwerveModulePIDConfig pidConfig) {
        this.motor = motor;
        this.encoder = encoder;
        BreakerCANCoderFactory.configExistingCANCoder(encoder, SensorInitializationStrategy.BootToAbsolutePosition,
                AbsoluteSensorRange.Signed_PlusMinus180, encoderAbsoluteAngleOffsetDegrees, false);

        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        turnConfig.remoteFilter0.remoteSensorDeviceID = encoder.getDeviceID();
        turnConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        turnConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        turnConfig.slot0.kP = pidConfig.kP;
        turnConfig.slot0.kI = pidConfig.kI;
        turnConfig.slot0.kD =pidConfig.kD;
        turnConfig.slot0.closedLoopPeakOutput = 1.0;
        turnConfig.peakOutputForward = 1.0;
        turnConfig.peakOutputReverse = -1.0;
        turnConfig.voltageCompSaturation = 12.0;
        turnConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 1.5);
        BreakerCTREUtil.checkError(motor.configAllSettings(turnConfig),
                " Failed to config swerve module turn motor ");
        motor.selectProfileSlot(0, 0);
        motor.setSensorPhase(true);
        motor.setInverted(isMotorInverted);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.set(ControlMode.Position, 0);
        targetAngle = new Rotation2d();
        deviceName = "TalonFX_Swerve_Angle_Motor_(" + motor.getDeviceID() + ")";
    }

    @Override
    public DevicePowerMode managePower(BreakerPowerManagementConfig managementConfig, double... managementPerameters) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void overrideAutomaticPowerManagement(DevicePowerMode manualPowerMode) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void returnToAutomaticPowerManagement() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isUnderAutomaticControl() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public DevicePowerMode getPowerMode() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void runSelfTest() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public DeviceHealth getHealth() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public String getFaults() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public String getDeviceName() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public boolean hasFault() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void setDeviceName(String newName) {
        deviceName = newName;
        
    }

    @Override
    public void setTargetAngle(Rotation2d targetAngle) {
        double relTgtAng = BreakerMath.absoluteAngleToContinuousRelativeAngleDegrees(getRelativeAngle(),
                Rotation2d.fromDegrees(getAblsoluteAngle()), targetAngle);
        motor.set(TalonFXControlMode.Position, BreakerUnits.degreesToCANCoderNativeUnits(relTgtAng));
        this.targetAngle = targetAngle;
        
    }

    @Override
    public double getAblsoluteAngle() {
        return encoder.getAbsolutePosition();
    }

    @Override
    public double getRelativeAngle() {
        return encoder.getPosition();
    }
    @Override
    public void setBrakeMode(boolean isEnabled) {
       motor.setNeutralMode(isEnabled ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }}
