// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.offseasionbot.non_subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.offseasionbot.non_subsystems.Node.NodeHeight;
import frc.robot.subsystems.offseasionbot.non_subsystems.Node.NodeCoulmn;
import frc.robot.subsystems.offseasionbot.non_subsystems.OffseasionBotConstants.ScoreingConstants;

import static frc.robot.subsystems.offseasionbot.non_subsystems.OffseasionBotConstants.OperatorConstants.*;

import java.util.Optional;

/** Add your docs here. */
public class OperatorController {
    private final GenericHID hid;
    private final JoystickButton leftNodeGroupButton, centerNodeGroupButton, rightNodeGroupButton;
    private final JoystickButton leftHighNodeButton, centerHighNodeButton, rightHighNodeButton;
    private final JoystickButton leftMidNodeButton, centerMidNodeButton, rightMidNodeButton;
    private final JoystickButton leftLowNodeButton, centerLowNodeButton, rightLowNodeButton;
    private final Trigger leftNodeSelectedTrigger, centerNodeSelectedTrigger, rightNodeSelectedTrigger;
    private final Trigger highNodeSelectedTrigger, midNodeSelectedTrigger, lowNodeSelectedTrigger;
    private final Trigger nodeHeightSelectedTrigger, nodeCoulmnSelectedTrigger;
    private final Trigger scoringCommandRequestTrigger, nodeGroupSelectedTrigger, inGroupNodeSelectedTrigger;
    public OperatorController(int port) {
        hid = new GenericHID(OPERATOR_PAD_PORT);
        leftNodeGroupButton = new JoystickButton(hid, );
        centerNodeGroupButton = new JoystickButton(hid, );
        rightNodeGroupButton = new JoystickButton(hid, );
      
        leftNodeSelectedTrigger = leftHighNodeButton.or(leftMidNodeButton).or(leftLowNodeButton);
        centerNodeSelectedTrigger = centerHighNodeButton.or(centerMidNodeButton).or(centerLowNodeButton);
        rightNodeSelectedTrigger = rightHighNodeButton.or(rightMidNodeButton).or(rightLowNodeButton);
        nodeCoulmnSelectedTrigger = leftNodeSelectedTrigger.or(centerNodeSelectedTrigger).or(rightNodeSelectedTrigger)

        highNodeSelectedTrigger = leftHighNodeButton.or(centerHighNodeButton).or(rightHighNodeButton);
        midNodeSelectedTrigger = leftMidNodeButton.or(centerMidNodeButton).or(rightMidNodeButton);
        lowNodeSelectedTrigger = leftLowNodeButton.or(centerLowNodeButton).or(rightLowNodeButton);
        nodeHeightSelectedTrigger = highNodeSelectedTrigger.or(midNodeSelectedTrigger).or(lowNodeSelectedTrigger);

        nodeGroupSelectedTrigger = leftNodeGroupButton.or(centerNodeGroupButton).or(rightNodeGroupButton);
        inGroupNodeSelectedTrigger = nodeCoulmnSelectedTrigger.and(nodeHeightSelectedTrigger);

        scoringCommandRequestTrigger = nodeGroupSelectedTrigger.and(inGroupNodeSelectedTrigger);
    }

    public JoystickButton getLeftNodeGroupButton() {
        return leftNodeGroupButton;
    }

    public JoystickButton getCenterNodeGroupButton() {
        return centerNodeGroupButton;
    }

    public JoystickButton getRightNodeGroupButton() {
        return rightNodeGroupButton;
    }

    public JoystickButton getLeftHighNodeButton() {
        return leftHighNodeButton;
    }

    public JoystickButton getCenterHighNodeButton() {
        return centerHighNodeButton;
    }

    public JoystickButton getRightHighNodeButton() {
        return rightHighNodeButton;
    }

    public JoystickButton getLeftMidNodeButton() {
        return leftMidNodeButton;
    }

    public JoystickButton getCenterMidNodeButton() {
        return centerMidNodeButton;
    }

    public JoystickButton getRightMidNodeButton() {
        return rightMidNodeButton;
    }

    public JoystickButton getLeftLowNodeButton() {
        return leftLowNodeButton;
    }

    public JoystickButton getCenterLowNodeButton() {
        return centerLowNodeButton;
    }

    public JoystickButton getRightLowNodeButton() {
        return rightLowNodeButton;
    }

    public Trigger getScoringCommandRequestTrigger() {
        return scoringCommandRequestTrigger;
    }

    public Optional<Node> getSelectedScoringNode() {
        int colOrd = 0;
        NodeHeight height = NodeHeight.HIGH;
        if (scoringCommandRequestTrigger.getAsBoolean()) {

            if (leftNodeGroupButton.getAsBoolean()) {
                colOrd = ScoreingConstants.LEFT_NODE_GROUP_CENTRAL_COULMN_ORDINAL;
            } else if (centerNodeGroupButton.getAsBoolean()) {
                colOrd = ScoreingConstants.CENTER_NODE_GROUP_CENTRAL_COULMN_ORDINAL;
            } else if (rightNodeGroupButton.getAsBoolean()) {
                colOrd = ScoreingConstants.RIGHT_NODE_GROUP_CENTRAL_COULMN_ORDINAL;
            } else {
                return Optional.empty();
            }

            if (highNodeSelectedTrigger.getAsBoolean()) {
                height = NodeHeight.HIGH;
            } else if (midNodeSelectedTrigger.getAsBoolean()) {
                height = NodeHeight.MID;
            } else if (lowNodeSelectedTrigger.getAsBoolean()) {
                height = NodeHeight.LOW;
            } else {
                return Optional.empty();
            }

            return Node.fromCoulmnAndHeight(NodeCoulmn.fromOrdinal(colOrd), height);
        }
        return Optional.empty();
        
    }





}
