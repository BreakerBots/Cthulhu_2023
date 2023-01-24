// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Arm extends SubsystemBase {
    private WPI_TalonFX sholderMotor, elbowMotor; 
    private ArmFeedforward shoulderFF, elbowFF;
    private ArmState targetState;
    
    public enum ArmState {
        PLACE_HIGH_CONE(0.0, 0.0),
        PLACE_HIGH_CUBE(0.0, 0.0),
        PLACE_MEDIUM_CONE(0.0, 0.0),
        PLACE_MEDIUM_CUBE(0.0, 0.0),
        PLACE_LOW(0.0, 0.0),
        PICKUP_HIGH(0.0, 0.0),
        PICKUP_LOW(0.0, 0.0),
        BASE_CARRY(0.0, 0.0),
        STOW_ARM(0.0, 0.0);

        ArmState(double shoulderAngle, double elbowAngle) {
            this.shoulderAngle = shoulderAngle;
            this.elbowAngle = elbowAngle;
        }

        public double shoulderAngle, elbowAngle;
    }

    public Arm() {
        sholderMotor = new WPI_TalonFX(0);
        elbowMotor = new WPI_TalonFX(0);
        shoulderFF = new ArmFeedforward(0, 0, 0);
        shoulderFF = new ArmFeedforward(0, 0, 0);
    }

    public void setTargetState(ArmState targetState) {
        this.targetState = targetState;
    }

    @Override
    public void periodic() {
        
    }
}
