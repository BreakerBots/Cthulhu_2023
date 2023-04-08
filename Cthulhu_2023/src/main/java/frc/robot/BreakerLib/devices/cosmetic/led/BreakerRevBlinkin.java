// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.BreakerLib.devices.BreakerGenericLoopedDevice;
import frc.robot.BreakerLib.devices.cosmetic.led.animations.BreakerAnimation;
import frc.robot.BreakerLib.util.BreakerRoboRIO.RobotOperatingMode;

/** Add your docs here. */
public class BreakerRevBlinkin {

    PWMSparkMax blinkin;

    public BreakerRevBlinkin(int channel) {
        blinkin = new PWMSparkMax(channel);
    }
}

enum PatternPallete {
    RAINBOW(0),
    PARTY(.02),
    OCEAN(.04),
    LAVA(.06),
    FOREST(.08);

    final double pwmVal;
    PatternPallete(double pwmVal) {
        this.pwmVal = pwmVal;
    }
}

enum PatternType {
    RAINBOW(-0.99),
    SINELON(-0.79),
    BEATS_PER_MIN(-0.69),
    TWINKLES(-0.55),
    COLOR_WAVES(-0.45);

    final double pwmVal;
    PatternType(double pwmVal) {
        this.pwmVal = pwmVal;
    }
}
