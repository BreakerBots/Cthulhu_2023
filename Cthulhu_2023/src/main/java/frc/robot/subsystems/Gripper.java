// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.management.relation.RelationService;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.GamePieceType;
import frc.robot.BreakerLib.devices.sensors.color.BreakerColorSensorV3;
import frc.robot.Constants;

/** Add your docs here. */
public class Gripper {
    private CANSparkMax spark;
    private SparkMaxPIDController pid;
    private RelativeEncoder encoder;
    public Gripper() {
        spark = new CANSparkMax(0, MotorType.kBrushless);
        spark.setSmartCurrentLimit(20);
        pid = spark.getPIDController();
        encoder = spark.getEncoder();
        encoder.setPosition(0);
        pid.setFeedbackDevice(encoder);
        pid.setP(0);
        pid.setI(0);
        pid.setD(0);
    }

    public double getGripperPosition() {
       return encoder.getPosition() * Constants.GripperConstants.MOTOR_ROT_TO_GRIP_POS_CM;
    }

    private void setGripperPosition(double position) {
        pid.setReference(position / Constants.GripperConstants.MOTOR_ROT_TO_GRIP_POS_CM, ControlType.kPosition);
    }

    public void close(GamePieceType type) {
        switch (type) {
            case CONE:
                setGripperPosition(Constants.GripperConstants.CONE_GRIP_POSITION);
                break;
            case CUBE:
                setGripperPosition(Constants.GripperConstants.CUBE_GRIP_POSITION);
                break;
            case NONE:
                setGripperPosition(0);
                break;     
        }
    }

    public void open() {
        setGripperPosition(Constants.GripperConstants.OPEN_GRIP_POSITION);
    }

    public Color getColorSensorDetectedColor() {
        return null;
    }

    public GamePieceType getControlledGamePieceType() {  
        return null;
    }

    
}
