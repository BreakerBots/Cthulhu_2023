// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.annotation.Nulls;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public final class Node {
    public static final Node[] NODES = new Node[] {
        new Node(NodeGroup.SUBSTATION, NodeGroup.SUBSTATION, NodeLevel.HIGH, new Translation3d()),
        new Node(NodeGroup.SUBSTATION, NodeGroup.CENTER, NodeLevel.HIGH, new Translation3d()),
        new Node(NodeGroup.SUBSTATION, NodeGroup.GATE, NodeLevel.HIGH, new Translation3d()),
        new Node(NodeGroup.CENTER, NodeGroup.SUBSTATION, NodeLevel.HIGH, new Translation3d()),
        new Node(NodeGroup.CENTER, NodeGroup.CENTER, NodeLevel.HIGH, new Translation3d()),
        new Node(NodeGroup.CENTER, NodeGroup.GATE, NodeLevel.HIGH, new Translation3d()),
        new Node(NodeGroup.GATE, NodeGroup.SUBSTATION, NodeLevel.HIGH, new Translation3d()),
        new Node(NodeGroup.GATE, NodeGroup.CENTER, NodeLevel.HIGH, new Translation3d()),
        new Node(NodeGroup.GATE, NodeGroup.GATE, NodeLevel.HIGH, new Translation3d()),
        new Node(NodeGroup.SUBSTATION, NodeGroup.SUBSTATION, NodeLevel.MIDDLE, new Translation3d()),
        new Node(NodeGroup.SUBSTATION, NodeGroup.CENTER, NodeLevel.MIDDLE, new Translation3d()),
        new Node(NodeGroup.SUBSTATION, NodeGroup.GATE, NodeLevel.MIDDLE, new Translation3d()),
        new Node(NodeGroup.CENTER, NodeGroup.SUBSTATION, NodeLevel.MIDDLE, new Translation3d()),
        new Node(NodeGroup.CENTER, NodeGroup.CENTER, NodeLevel.MIDDLE, new Translation3d()),
        new Node(NodeGroup.CENTER, NodeGroup.GATE, NodeLevel.MIDDLE, new Translation3d()),
        new Node(NodeGroup.GATE, NodeGroup.SUBSTATION, NodeLevel.MIDDLE, new Translation3d()),
        new Node(NodeGroup.GATE, NodeGroup.CENTER, NodeLevel.MIDDLE, new Translation3d()),
        new Node(NodeGroup.GATE, NodeGroup.GATE, NodeLevel.MIDDLE, new Translation3d()),
        new Node(NodeGroup.SUBSTATION, NodeGroup.SUBSTATION, NodeLevel.LOW, new Translation3d()),
        new Node(NodeGroup.SUBSTATION, NodeGroup.CENTER, NodeLevel.LOW, new Translation3d()),
        new Node(NodeGroup.SUBSTATION, NodeGroup.GATE, NodeLevel.LOW, new Translation3d()),
        new Node(NodeGroup.CENTER, NodeGroup.SUBSTATION, NodeLevel.LOW, new Translation3d()),
        new Node(NodeGroup.CENTER, NodeGroup.CENTER, NodeLevel.LOW, new Translation3d()),
        new Node(NodeGroup.CENTER, NodeGroup.GATE, NodeLevel.LOW, new Translation3d()),
        new Node(NodeGroup.GATE, NodeGroup.SUBSTATION, NodeLevel.LOW, new Translation3d()),
        new Node(NodeGroup.GATE, NodeGroup.CENTER, NodeLevel.LOW, new Translation3d()),
        new Node(NodeGroup.GATE, NodeGroup.GATE, NodeLevel.LOW, new Translation3d())
    };
    
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

    private NodeGroup group, subGroup;
    private NodeLevel level;
    private Translation3d bluePos;

    private Node(NodeGroup group, NodeGroup subGroup, NodeLevel level, Translation3d bluePos) {
        this.group = group;
        this.subGroup = subGroup;
        this.level = level;
    }

    public NodeGroup getGroup() {
        return group;
    }

    public NodeGroup getSubGroup() {
        return subGroup;
    }

    public NodeLevel getLevel() {
        return level;
    }

    public Translation3d getPosition() {
        double aos = (16.54 / 2);
        return DriverStation.getAlliance() == Alliance.Red ? new Translation3d(aos + (aos - bluePos.getX()), bluePos.getY(), bluePos.getZ()) : bluePos;
    }

    public static Node fromGroups(NodeGroup group, NodeGroup subGroup) {
        for (Node node: NODES) {
            if (node.group == group && node.subGroup == subGroup) {
                return node;
            }
        }
        return null;
    }

    @Override
    public boolean equals(Object obj) {
        Node other = (Node) obj;
        return group == other.group && subGroup == other.subGroup && level == other.level; 
    }
}
