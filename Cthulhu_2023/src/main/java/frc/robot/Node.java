// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.arm.Arm.ArmState;

/** Add your docs here. */
public final class Node {

    public enum NodeType {
        CONE,
        CUBE,
        HYBRID
    }
    
    /** 
     * Substation = Tags 3 & 6
     * Center = Coopertition Nodes, Tags 2 & 7
     * Gate = Tags 1 & 8
     */
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

    /** Order is Substation -> Center -> Gate, with suborder Substation -> Center -> Gate. Poses apply for Blue Alliance. */
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

    /** Order is High -> Middle -> Low, Columns index 0 to 8, and respective node type info. */
    public static final Node[] NODES = new Node[] {
        new Node(NodeLevel.HIGH, COLUMNS[0], NodeType.CONE, ArmState.PLACE_HIGH_CONE),
        new Node(NodeLevel.HIGH, COLUMNS[1], NodeType.CUBE, ArmState.PLACE_HIGH_CUBE),
        new Node(NodeLevel.HIGH, COLUMNS[2], NodeType.CONE, ArmState.PLACE_HIGH_CONE),
        new Node(NodeLevel.HIGH, COLUMNS[3], NodeType.CONE, ArmState.PLACE_HIGH_CONE),
        new Node(NodeLevel.HIGH, COLUMNS[4], NodeType.CUBE, ArmState.PLACE_HIGH_CUBE),
        new Node(NodeLevel.HIGH, COLUMNS[5], NodeType.CONE, ArmState.PLACE_HIGH_CONE),
        new Node(NodeLevel.HIGH, COLUMNS[6], NodeType.CONE, ArmState.PLACE_HIGH_CONE),
        new Node(NodeLevel.HIGH, COLUMNS[7], NodeType.CUBE, ArmState.PLACE_HIGH_CUBE),
        new Node(NodeLevel.HIGH, COLUMNS[8], NodeType.CONE, ArmState.PLACE_HIGH_CONE),
        new Node(NodeLevel.MIDDLE, COLUMNS[0], NodeType.CONE, ArmState.PLACE_MEDIUM_CONE),
        new Node(NodeLevel.MIDDLE, COLUMNS[1], NodeType.CUBE, ArmState.PLACE_MEDIUM_CUBE),
        new Node(NodeLevel.MIDDLE, COLUMNS[2], NodeType.CONE, ArmState.PLACE_MEDIUM_CONE),
        new Node(NodeLevel.MIDDLE, COLUMNS[3], NodeType.CONE, ArmState.PLACE_MEDIUM_CONE),
        new Node(NodeLevel.MIDDLE, COLUMNS[4], NodeType.CUBE, ArmState.PLACE_MEDIUM_CUBE),
        new Node(NodeLevel.MIDDLE, COLUMNS[5], NodeType.CONE, ArmState.PLACE_MEDIUM_CONE),
        new Node(NodeLevel.MIDDLE, COLUMNS[6], NodeType.CONE, ArmState.PLACE_MEDIUM_CONE),
        new Node(NodeLevel.MIDDLE, COLUMNS[7], NodeType.CUBE, ArmState.PLACE_MEDIUM_CUBE),
        new Node(NodeLevel.MIDDLE, COLUMNS[8], NodeType.CONE, ArmState.PLACE_MEDIUM_CONE),
        new Node(NodeLevel.LOW, COLUMNS[0], NodeType.HYBRID, ArmState.PLACE_HYBRID),
        new Node(NodeLevel.LOW, COLUMNS[1], NodeType.HYBRID, ArmState.PLACE_HYBRID),
        new Node(NodeLevel.LOW, COLUMNS[2], NodeType.HYBRID, ArmState.PLACE_HYBRID),
        new Node(NodeLevel.LOW, COLUMNS[3], NodeType.HYBRID, ArmState.PLACE_HYBRID),
        new Node(NodeLevel.LOW, COLUMNS[4], NodeType.HYBRID, ArmState.PLACE_HYBRID),
        new Node(NodeLevel.LOW, COLUMNS[5], NodeType.HYBRID, ArmState.PLACE_HYBRID),
        new Node(NodeLevel.LOW, COLUMNS[6], NodeType.HYBRID, ArmState.PLACE_HYBRID),
        new Node(NodeLevel.LOW, COLUMNS[7], NodeType.HYBRID, ArmState.PLACE_HYBRID),
        new Node(NodeLevel.LOW, COLUMNS[8], NodeType.HYBRID, ArmState.PLACE_HYBRID),

    };

    private NodeColumn column;
    private NodeLevel level;
    private NodeType type;
    private ArmState placementState;

    private Node(NodeLevel level, NodeColumn column, NodeType type, ArmState placementState) {
        this.column = column;
        this.level = level;
        this.type = type;
        this.placementState = placementState;
    }

    public NodeColumn getColumn() {
        return column;
    }

    public NodeLevel getLevel() {
        return level;
    }

    public NodeType getNodeType() {
        return type;
    }

    public ArmState getPlacementState() {
        return placementState;
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
