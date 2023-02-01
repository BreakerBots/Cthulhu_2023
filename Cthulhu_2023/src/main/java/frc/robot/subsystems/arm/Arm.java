// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Add your docs here. */
public class Arm extends SubsystemBase {
    private ArmJoint shoulderJoint, elbowJoint;
    private ArmState targetState = ArmState.UNKNOWN;

    
    public enum ArmState {
        PLACE_HIGH_CONE(0.0, 0.0),
        PLACE_HIGH_CUBE(0.0, 0.0),
        PLACE_MEDIUM_CONE(0.0, 0.0),
        PLACE_MEDIUM_CUBE(0.0, 0.0),
        PLACE_LOW(0.0, 0.0),
        PICKUP_HIGH(0.0, 0.0),
        PICKUP_LOW(0.0, 0.0),
        CARRY(0.0, 0.0),
        STOW_ARM(0.0, 0.0),
        UNKNOWN(0.0, 0.0);

        ArmState(double shoulderAngleDeg, double elbowAngleDeg) {
            statePose = new ArmPose(Rotation2d.fromDegrees(shoulderAngleDeg), Rotation2d.fromDegrees(elbowAngleDeg));
        }

        public ArmPose statePose;
    }

    public static class ArmPose {
        private Rotation2d shoulderAngle, elbowAngle;
        public ArmPose(Rotation2d shoulderAngle, Rotation2d elbowAngle) {
            this.shoulderAngle = shoulderAngle;
            this.elbowAngle = elbowAngle;
        }

        public Rotation2d getElbowAngle() {
            return elbowAngle;
        }

        public Rotation2d getShoulderAngle() {
            return shoulderAngle;
        }
    }

    public Arm() {
        ArmJoint.ArmJointConfig shoulderConfig = new ArmJoint.ArmJointConfig(
            new WPI_TalonFX(0), 
            new WPI_CANCoder(0), 0, false, 
            new TrapezoidProfile.Constraints(0,0), 
            0, 0, 0, 
            0, 0, 0, 0
        );

        ArmJoint.ArmJointConfig elbowConfig = new ArmJoint.ArmJointConfig(
            new WPI_TalonFX(0), 
            new WPI_CANCoder(0), 0, false, 
            new TrapezoidProfile.Constraints(0,0), 
            0, 0, 0, 
            0, 0, 0, 0
        );
        shoulderJoint = new ArmJoint(() -> {return new Rotation2d();}, shoulderConfig);
        elbowJoint = new ArmJoint(shoulderJoint::getJointAngle, elbowConfig);
    }

    public void setTargetState(ArmState targetState) {
        this.targetState = targetState;
        shoulderJoint.setGoal(targetState.statePose.shoulderAngle.getRadians()); 
        elbowJoint.setGoal(targetState.statePose.elbowAngle.getRadians()); 
    }

    public ArmState getTargetState() {
        return targetState;
    }

    public ArmPose getArmPose() {
        return new ArmPose(shoulderJoint.getJointAngle(), elbowJoint.getJointAngle());
    }

    public boolean atTargetState() {
        ArmPose curPose = getArmPose();
        return  BreakerMath.lambdaEquals(targetState.statePose.shoulderAngle.getDegrees(), curPose.shoulderAngle.getDegrees(), 1.5) &&
                BreakerMath.lambdaEquals(targetState.statePose.elbowAngle.getDegrees(), curPose.elbowAngle.getDegrees(), 1.5);
    }

    @Override
    public void periodic() {
        
    }
}
