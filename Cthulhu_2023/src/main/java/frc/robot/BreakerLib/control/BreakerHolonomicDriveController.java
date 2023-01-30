// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.control;

import java.util.Currency;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;

/** Add your docs here. */
public class BreakerHolonomicDriveController {
    private PIDController linearController;
    private ProfiledPIDController angleController;
    private Pose2d curPose = new Pose2d(), tgtPose = new Pose2d(), tolerences = new Pose2d();
    private boolean calculateHasBeenRun = false;

    public BreakerHolonomicDriveController(PIDController linearController, ProfiledPIDController angleController) {
        this.linearController = linearController;
        this.angleController = angleController;
    }

    public void setTolerances(Pose2d tolerences) {
        this.tolerences = tolerences;
    }

    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d targetPose, double maxLinearVelocity) {
        Translation2d errTrans = targetPose.getTranslation().minus(currentPose.getTranslation());
        double tgtVel = -linearController.calculate(errTrans.getNorm(), 0);
        BreakerVector2 vec = BreakerVector2.fromTranslation(errTrans).getUnitVector().times(MathUtil.clamp(tgtVel, -maxLinearVelocity, maxLinearVelocity));
        curPose = currentPose;
        tgtPose = targetPose;
        calculateHasBeenRun = true;
        return new ChassisSpeeds(vec.getMagnitudeX(), vec.getMagnitudeY(), angleController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()));
    }

    public boolean atTargetPose() {
        return  Math.abs(tgtPose.getX() - curPose.getX()) <= tolerences.getX() &&
                Math.abs(tgtPose.getY() - curPose.getY()) <= tolerences.getY() &&
                Math.abs(tgtPose.getRotation().getRadians() - curPose.getRotation().getRadians()) <= tolerences.getRotation().getRadians() &&
                calculateHasBeenRun;
    }


}