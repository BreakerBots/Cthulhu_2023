
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.offseasionbot.non_subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;

 /** Add your docs here. */
public enum Node {
    C0_L(NodeCoulmn.C0, NodeHeight.LOW, NodeType.HYBRID),
    C0_M(NodeCoulmn.C0, NodeHeight.MID, NodeType.CONE),
    C0_H(NodeCoulmn.C0, NodeHeight.HIGH, NodeType.CONE);

    private NodeCoulmn coulmn;
    private NodeHeight height; 
    private NodeType type;
    private Node(NodeCoulmn coulmn, NodeHeight height, NodeType type) {
        this.coulmn = coulmn;
        this.height = height;
        this.type = type;
    }

    public NodeCoulmn getCoulmn() {
        return coulmn;
    }

    public NodeHeight getHeight() {
        return height;
    }

    public NodeType getType() {
        return type;
    }

    public Pose2d getAllignmentPose() {
        return new Pose2d(coulmn.getBlueBaseAllignmentPose().getX() + height.getAllignmentOffset(), coulmn.getBlueBaseAllignmentPose().getY(), coulmn.getBlueBaseAllignmentPose().getRotation());
    }
 
    public static Optional<Node> fromCoulmnAndHeight(NodeCoulmn tgtCoulmn, NodeHeight tgtHeight) {
        for (Node node : values()) {
            if (node.getCoulmn() == tgtCoulmn && node.getHeight() == tgtHeight) {
                return Optional.of(node);
            }
        }
        return Optional.empty();
    }
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

        public Pose2d getBlueBaseAllignmentPose() {
            return blueBaseAllignmentPose;
        }

        public static NodeCoulmn fromOrdinal(int ordinal) {
            for (NodeCoulmn col : NodeCoulmn.values()) {
                if (col.ordinal() == ordinal) {
                    return col;
                }
            }
            return null;
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
