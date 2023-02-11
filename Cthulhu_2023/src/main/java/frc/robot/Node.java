// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import com.fasterxml.jackson.annotation.Nulls;

// import edu.wpi.first.math.geometry.Translation3d;

// /** Add your docs here. */
// public enum Node {
//     H();
//     public enum NodeGroup {
//         SUBSTATION,
//         CENTER,
//         GATE
//     }

//     public enum NodeLevel {
//         HIGH,
//         MIDDLE,
//         HYBRID
//     }
//     private NodeGroup group, subGroup;
//     private NodeLevel level;
//     private int num; 
//     private Translation3d redPos, bluePos;
//     private Node(NodeGroup group, NodeGroup subGroup, NodeLevel level, int num, Translation3d redPos, Translation3d bluePos) {
//         this.group = group;
//         this.subGroup = subGroup;
//         this.level = level;
//     }

//     public NodeGroup getGroup() {
//         return group;
//     }

//     public NodeGroup getSubGroup() {
//         return subGroup;
//     }

//     public NodeLevel getLevel() {
//         return level;
//     }
// }
