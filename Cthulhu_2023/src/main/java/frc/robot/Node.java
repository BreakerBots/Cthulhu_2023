// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.text.html.HTMLDocument.BlockElement;

import com.fasterxml.jackson.annotation.Nulls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public final class Node {
    public static final NodeColumn[] COLUMNS = new NodeColumn[] {
        new NodeColumn(NodeGroup.SUBSTATION, NodeGroup.SUBSTATION, new Pose2d()),
        new NodeColumn(NodeGroup.SUBSTATION, NodeGroup.CENTER, new Pose2d()),
        new NodeColumn(NodeGroup.SUBSTATION, NodeGroup.GATE, new Pose2d()),
        new NodeColumn(NodeGroup.CENTER, NodeGroup.SUBSTATION, new Pose2d()),
        new NodeColumn(NodeGroup.CENTER, NodeGroup.CENTER, new Pose2d()),
        new NodeColumn(NodeGroup.CENTER, NodeGroup.GATE, new Pose2d()),
        new NodeColumn(NodeGroup.GATE, NodeGroup.SUBSTATION, new Pose2d()),
        new NodeColumn(NodeGroup.GATE, NodeGroup.CENTER, new Pose2d()),
        new NodeColumn(NodeGroup.GATE, NodeGroup.GATE, new Pose2d())
    };

    public static final Node[] NODES = new Node[] {
        new Node(NodeLevel.HIGH, COLUMNS[0]),
        new Node(NodeLevel.HIGH, COLUMNS[1]),
        new Node(NodeLevel.HIGH, COLUMNS[2]),
        new Node(NodeLevel.HIGH, COLUMNS[3]),
        new Node(NodeLevel.HIGH, COLUMNS[4]),
        new Node(NodeLevel.HIGH, COLUMNS[5]),
        new Node(NodeLevel.HIGH, COLUMNS[6]),
        new Node(NodeLevel.HIGH, COLUMNS[7]),
        new Node(NodeLevel.HIGH, COLUMNS[8]),
        new Node(NodeLevel.MIDDLE, COLUMNS[0]),
        new Node(NodeLevel.MIDDLE, COLUMNS[1]),
        new Node(NodeLevel.MIDDLE, COLUMNS[2]),
        new Node(NodeLevel.MIDDLE, COLUMNS[3]),
        new Node(NodeLevel.MIDDLE, COLUMNS[4]),
        new Node(NodeLevel.MIDDLE, COLUMNS[5]),
        new Node(NodeLevel.MIDDLE, COLUMNS[6]),
        new Node(NodeLevel.MIDDLE, COLUMNS[7]),
        new Node(NodeLevel.MIDDLE, COLUMNS[8]),
        new Node(NodeLevel.LOW, COLUMNS[0]),
        new Node(NodeLevel.LOW, COLUMNS[1]),
        new Node(NodeLevel.LOW, COLUMNS[2]),
        new Node(NodeLevel.LOW, COLUMNS[3]),
        new Node(NodeLevel.LOW, COLUMNS[4]),
        new Node(NodeLevel.LOW, COLUMNS[5]),
        new Node(NodeLevel.LOW, COLUMNS[6]),
        new Node(NodeLevel.LOW, COLUMNS[7]),
        new Node(NodeLevel.LOW, COLUMNS[8]),

    };

    public static final class NodeColumn {
        private NodeGroup group, subGroup;
        private Pose2d blueAlignmentPose;
        public NodeColumn(NodeGroup group, NodeGroup subGroup, Pose2d blueAlignmentPose) {
            this.group = group;
            this.subGroup = subGroup;
            this.blueAlignmentPose = blueAlignmentPose;
        }

        public NodeGroup getNodeGroup() {
            return group;
        }

        public NodeGroup getNodeSubGroup() {
            return subGroup;
        }

        public Pose2d getAlignmentPose() {
            double aos = (16.54 / 2);
            return DriverStation.getAlliance() == Alliance.Red ? new Pose2d(aos + (aos - blueAlignmentPose.getX()), blueAlignmentPose.getY(), Rotation2d.fromDegrees(180)) : blueAlignmentPose;
        }

        @Override
        public boolean equals(Object obj) {
            NodeColumn other = (NodeColumn) obj;
            return group == other.group && subGroup == other.subGroup;
        }
    }
    
    public enum NodeGroup {
        SUBSTATION,
        CENTER,
        GATE
    }

    public enum NodeLevel {
        HIGH,
        MIDDLE,
        LOW
    }

    private NodeColumn column;
    private NodeLevel level;
    private Translation3d bluePos;

    private Node(NodeLevel level, NodeColumn column) {
        this.column = column;
        this.level = level;
    }

    public NodeColumn getColumn() {
        return column;
    }

    public NodeLevel getLevel() {
        return level;
    }

    public static Node fromGroups(NodeGroup group, NodeGroup subGroup) {
        for (Node node: NODES) {
            if (node.column.group == group && node.column.subGroup == subGroup) {
                return node;
            }
        }
        return null;
    }

    public static Node fromColumnAndLevel(NodeColumn column, NodeLevel level) {
        for (Node node: NODES) {
            if (node.column.equals(column) && node.level == level) {
                return node;
            }
        }
        return null;
    }

    @Override
    public boolean equals(Object obj) {
        Node other = (Node) obj;
        return column.equals(other.column) && level == other.level; 
    }
}
