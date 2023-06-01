// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class BreakerFalconArm implements BreakerRotationArm {

    private WPI_TalonFX motor;
    private WPI_CANCoder encoder;
    private ArmFeedforward armFF;
    private ProfiledPIDController pid;

    public BreakerFalconArm(WPI_TalonFX motor, WPI_CANCoder encoder, ArmFeedforward armFF, ProfiledPIDController pid) {
        this.motor = motor;
        this.encoder = encoder;
        this.armFF = armFF;
        this.pid = pid;
    }

    public void setTargetAngle(double degrees) {
        pid.setGoal(degrees);
    }

    public void setTargetState(TrapezoidProfile.State tgtState) {
        pid.setGoal(tgtState);
    }

    public void setConstraints(TrapezoidProfile.Constraints constraints) {
        pid.setConstraints(constraints);
    }

    public void periodic() {
        TrapezoidProfile.State currentSetpoint = pid.getSetpoint();
        double position = Units.degreesToRadians(currentSetpoint.position);
        double velocity = Units.degreesToRadians(currentSetpoint.velocity);
        motor.setVoltage(armFF.calculate(position, velocity) + pid.calculate(encoder.getPosition()));
    }

}
