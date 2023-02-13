// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.GamePieceType;
import frc.robot.BreakerLib.devices.sensors.color.BreakerColorSensorV3;

/** Add your docs here. */
public class Gripper {
    private BreakerColorSensorV3 colorSensor;  //Todo: look into I2C lockup issue
    private float[] coneColorRangeMin = new float[]{0,0,0}; //Each val checked seprately
    private float[] coneColorRangeMax = new float[]{0,0,0}; //Each val checked seprately
    private float[] cubeColorRangeMin = new float[]{0,0,0}; //Each val checked seprately
    private float[] cubeColorRangeMax = new float[]{0,0,0}; //Each val checked seprately
    private double colorSensorProxMin = 0.5;
    public Gripper() {
        colorSensor = new BreakerColorSensorV3(null);
        
    }

    public void intake(GamePieceType type) {

    }

    public void dropGamePiece() {
        
    }

    public Color getColorSensorDetectedColor() {
        return colorSensor.getColor();
    }

    public GamePieceType getControlledGamePiece() {
        if (colorSensor.getProximity() >= colorSensorProxMin) {
            if (isColorInRange(getColorSensorDetectedColor(), coneColorRangeMin, coneColorRangeMax)) {
                return GamePieceType.CONE;
            } else if (isColorInRange(getColorSensorDetectedColor(), cubeColorRangeMin, cubeColorRangeMax)) {
                return GamePieceType.CUBE;
            }
        }
        return GamePieceType.NONE;
    }

    private boolean isColorInRange(Color color, float[] minHSB, float[] maxHSB) {
        Color8Bit color8B = new Color8Bit(color);
        float[] colorHSB = java.awt.Color.RGBtoHSB(color8B.red, color8B.green, color8B.blue, new float[3]);
        return  (minHSB[0] <= colorHSB[0] && colorHSB[0] <= maxHSB[0]) && 
                (minHSB[1] <= colorHSB[1] && colorHSB[1] <= maxHSB[1]) && 
                (minHSB[2] <= colorHSB[2] && colorHSB[2] <= maxHSB[2]);
    }

    
}
