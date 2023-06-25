// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.offseasionbot;

import frc.robot.subsystems.offseasionbot.Intake.ActuatorMotorState;

/** Add your docs here. */
public enum ElevatorIntakeAssemblyState {
    PLACE_HYBRID(0.0, ActuatorMotorState.EXTENDING),
    PLACE_CONE_MID(0.0, ActuatorMotorState.EXTENDING),
    PLACE_CONE_HIGH(0.0, ActuatorMotorState.EXTENDING),
    PLACE_CUBE_MID(0.0, ActuatorMotorState.EXTENDING),
    PLACE_CUBE_HIGH(0.0, ActuatorMotorState.EXTENDING),
    PICKUP_GROUND(0.0, ActuatorMotorState.EXTENDING),
    PICKUP_SINGLE_SUBSTATION(0.0, ActuatorMotorState.EXTENDING),
    PICKUP_DOUBLE_SUBSTATION(0.0, ActuatorMotorState.EXTENDING),
    STOW(0.0, ActuatorMotorState.RETRACTING);

    private final double elevatorTargetHeight;
    private final ActuatorMotorState intakeActuatorMotorState;
    private ElevatorIntakeAssemblyState(double elevatorTargetHeight, ActuatorMotorState intakeActuatorMotorState) {
        this.elevatorTargetHeight = elevatorTargetHeight;
        this.intakeActuatorMotorState = intakeActuatorMotorState;
    }

    public double getElevatorTargetHeight() {
        return elevatorTargetHeight;
    }

    public ActuatorMotorState getIntakeActuatorMotorState() {
        return intakeActuatorMotorState;
    }
}
