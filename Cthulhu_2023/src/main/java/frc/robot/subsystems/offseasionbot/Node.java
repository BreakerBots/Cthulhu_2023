// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.offseasionbot;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public enum Node {
    C0_L,
    C0_M,
    C0_H;
 
    public enum NodeCoulmn {
        C0(new Pose2d()),
        C1(new Pose2d()),
        C2(new Pose2d()),
        C3(new Pose2d()),
        C4(new Pose2d()),
        C5(new Pose2d()),
        C6(new Pose2d()),
        C7(new Pose2d()),
        C8(new Pose2d());
        private Pose2d blueBaseAllignmentPose;
        private NodeCoulmn(Pose2d blueBaseAllignmentPose) {
            this.blueBaseAllignmentPose = blueBaseAllignmentPose;
        }
    }

    public enum NodeHeight {
        LOW(0.0),
        MID(0.0),
        HIGH(0.0);

        private double allignmentOffset;
        private NodeHeight(double allignmentOffset) {
            this.allignmentOffset = allignmentOffset;
        }

        public double getAllignmentOffset() {
            return allignmentOffset;
        }
    }

    public enum NodeType {
        HYBRID,
        CONE,
        CUBE
    }



}
