// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.arm;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.BreakerLib.util.BreakerRoboRIO;
import frc.robot.BreakerLib.util.BreakerRoboRIO.RobotOperatingMode;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.MiscConstants.CANIVORE_1;

import java.util.Map;

/** Add your docs here. */
public class BreakerFalconArm extends ProfiledPIDSubsystem {

    public static final double kS = 0.0;
    public static final double kG = 0.675;
    public static final double kV = 0;
    public static final double kA = 0.0;
    public static final double kP = 0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2, 0.5);

    private WPI_TalonFX motor0 = new WPI_TalonFX(MAIN_MOTOR_ID, CANIVORE_1);
    private WPI_TalonFX motor1 = new WPI_TalonFX(SUB_MOTOR_ID, CANIVORE_1);
    private WPI_CANCoder encoder = new WPI_CANCoder(ARM_CANCODER_ID);
    private ArmFeedforward armFF = new ArmFeedforward(kS, kG, kV, kA);
    private static ProfiledPIDController pid = new ProfiledPIDController(kP, kI, kD, constraints);

    public BreakerFalconArm() {

        super(pid);
        enable();
        setGoal(getMeasurement());
        motor1.follow(motor0);

        motor0.setNeutralMode(NeutralMode.Coast);
        motor1.setNeutralMode(NeutralMode.Coast);

        motor1.setStatusFramePeriod(0, 0, 0);
        motor0.setInverted(TalonFXInvertType.Clockwise);
        motor1.setInverted(TalonFXInvertType.FollowMaster);
        motor0.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        motor0.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

        encoder.configSensorDirection(false);
        encoder.configMagnetOffset(ARM_CANCODER_OFFSET);
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    }

    @Override
    protected void useOutput(double output, State setpoint) {
        double ff = armFF.calculate(setpoint.position, setpoint.velocity);
        SmartDashboard.putNumber("Motor In", ff + output);
        motor0.setVoltage(ff + output);
    }

    @Override
    protected double getMeasurement() {
        return getPosRad();
    }

    public double getPosDeg() {
        return encoder.getAbsolutePosition()
                + (encoder.getAbsolutePosition() <= -90 && encoder.getAbsolutePosition() >= -180 ? 360 : 0);
    }

    public double getPosRad() {
        return Units.degreesToRadians(getPosDeg());
    }

    public void periodic() {
        SmartDashboard.putNumber("Current Angle", getPosDeg());
        SmartDashboard.putNumber("Target Angle", m_controller.getGoal().position);
        SmartDashboard.putNumber("Motor Out", motor0.getMotorOutputVoltage());

        if (m_enabled) {
            useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
        }
    }

}
