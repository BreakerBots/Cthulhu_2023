// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.yousif;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.math.BreakerMath;

public class YousifArm extends SubsystemBase {

  YousifArmJoint[] joints;

  public YousifArm(YousifArmJoint... joints) {

  }

  public YousifArmJoint[] getJoints() {
    return joints;
  }

  /** Assumes +- 180 deg for CANCoders. Angles fed in should be absolute, with 0 degrees being parallel to the ground and forward.*/
  public void moveJointsWithAbsoluteAngles(double... goalAngles) {
    joints[0].setGoal(goalAngles[0]);
    for (int i = 1; i < joints.length; i++) {
      joints[i].setGoal(BreakerMath.angleModulus(goalAngles[i] - goalAngles[i-1]));
    }
  }
}
