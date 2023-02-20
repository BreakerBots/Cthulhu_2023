// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Objects;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.SystemDiagnostics;
import frc.robot.BreakerLib.util.vendorutil.BreakerCTREUtil;

/** Add your docs here. */
public class Arm extends SubsystemBase {
    private ArmJoint shoulderJoint, elbowJoint;
    private ArmState targetState = ArmState.MANUAL;
    private ArmState prevState = ArmState.MANUAL;
    private ArmPose targetPose;
    private MoveToState activeMoveCommand;
    private WPI_CANCoder shoulderEncoder, elbowEncoder;
    private WPI_TalonFX shoulderMotor, elbowMotor;
    private SystemDiagnostics shoulderDiagnostics, elbowDiagnostics;
    
    public enum ArmState {
        PLACE_HIGH_CONE(12.0, 74.0),
        PLACE_HIGH_CUBE(15.0, 83.0),
        PLACE_MEDIUM_CONE(-9.0, 97.5),
        PLACE_MEDIUM_CUBE(0.0, 128.0),
        PLACE_HYBRID(18.0, 150.0),
        PICKUP_HIGH(15.0, 78.0),
        PICKUP_LOW(18.0, 150.0),
        CARRY(0.0, 170.0),
        STOW_ARM(-15.0, 170.0),
        MANUAL(0.0, 0.0); // Always zero
        
        private final ArmPose statePose;
        private final ArrayList<ArmPose> intermediaryPoses;

        ArmState(double shoulderAngleDeg, double elbowAngleDeg, ArmPose... intermedairyPoses) {
            statePose = new ArmPose(Rotation2d.fromDegrees(shoulderAngleDeg), Rotation2d.fromDegrees(elbowAngleDeg));
            this.intermediaryPoses = new ArrayList<>();
            for (ArmPose ap: intermedairyPoses) {
                this.intermediaryPoses.add(ap);
            }
        }

        public ArrayList<ArmPose> getIntermediaryPoses() {
            return new ArrayList<>(intermediaryPoses);
        }

        public ArmPose getStatePose() {
            return statePose;
        }
    }

    public static class ArmPose {
        private Rotation2d shoulderAngle, elbowAngle;
        public ArmPose(Rotation2d shoulderAngle, Rotation2d elbowAngle) {
            this.shoulderAngle = shoulderAngle;
            this.elbowAngle = elbowAngle;
        }

        public Rotation2d getElbowAngle() {
            return elbowAngle;
        }

        public Rotation2d getShoulderAngle() {
            return shoulderAngle;
        }

        @Override
        public boolean equals(Object obj) {
            ArmPose other = (ArmPose) obj;
            return BreakerMath.epsilonEquals(shoulderAngle.getDegrees(), other.shoulderAngle.getDegrees(), 1.5) &&
            BreakerMath.epsilonEquals(elbowAngle.getDegrees(), other.elbowAngle.getDegrees(), 1.5);
        }
    }

    public Arm() {
        shoulderEncoder = new WPI_CANCoder(0);
        shoulderMotor = new WPI_TalonFX(0);
        ArmJoint.ArmJointConfig shoulderConfig = new ArmJoint.ArmJointConfig( 
            new WPI_CANCoder(0), 0, false, 
            new TrapezoidProfile.Constraints(0,0), 
            0, 0, 0, 
            0, 0, 0, 0,
            new WPI_TalonFX(0)
        );

        elbowEncoder = new WPI_CANCoder(0);
        elbowMotor = new WPI_TalonFX(0);
        ArmJoint.ArmJointConfig elbowConfig = new ArmJoint.ArmJointConfig(
            new WPI_CANCoder(0), 0, false, 
            new TrapezoidProfile.Constraints(0,0), 
            0, 0, 0, 
            0, 0, 0, 0,
            new WPI_TalonFX(0)
        );
        shoulderJoint = new ArmJoint(() -> {return new Rotation2d();}, shoulderConfig);
        elbowJoint = new ArmJoint(shoulderJoint::getJointAngle, elbowConfig);

        shoulderDiagnostics = new SystemDiagnostics("Proximal_Arm");
        shoulderDiagnostics.addCTREMotorController(shoulderMotor);
        shoulderDiagnostics.addSupplier(() -> BreakerCTREUtil.checkCANCoderFaultsAndConnection(shoulderEncoder));
        elbowDiagnostics = new SystemDiagnostics("Distal_Arm");
        elbowDiagnostics.addCTREMotorController(elbowMotor);
        elbowDiagnostics.addSupplier(() -> BreakerCTREUtil.checkCANCoderFaultsAndConnection(elbowEncoder));
        targetPose = new ArmPose(shoulderJoint.getJointAngle(), elbowJoint.getJointAngle());
    }

    private void setTargetState(ArmState targetState) {
        prevState = this.targetState;
        this.targetState = targetState;
    }

    public void setManualTargetPose(ArmPose targetPose) {
        setTargetState(ArmState.MANUAL);
        setTargetPose(targetPose);
    }

    private void setTargetPose(ArmPose targetPose) {
        this.targetPose = targetPose;
        shoulderJoint.setGoal(targetPose.shoulderAngle.getRadians());
        elbowJoint.setGoal(targetPose.elbowAngle.getRadians());
    }

    public ArmState getPrevState() {
        return prevState;
    }

    public ArmState getTargetState() {
        return targetState;
    }

    public ArmPose getTargetPose() {
        return targetPose;
    }

    public ArmPose getArmPose() {
        return new ArmPose(shoulderJoint.getJointAngle(), elbowJoint.getJointAngle());
    }

    public boolean atTargetState() {
        ArmPose curPose = getArmPose();
        return  targetState.statePose.equals(curPose) && elbowJoint.getJointVel() < Math.toRadians(2) && shoulderJoint.getJointVel() < Math.toRadians(2);
    }

    public boolean atTargetPoseExact() {
        ArmPose curPose = getArmPose();
        return  targetPose.equals(curPose) && elbowJoint.getJointVel() < Math.toRadians(2) && shoulderJoint.getJointVel() < Math.toRadians(2);
    }

    public boolean atTargetPose() {
        ArmPose curPose = getArmPose();
        return  targetPose.equals(curPose);
    }

    @Override
    public void periodic() {
       
    }

    private void regesterMoveCommand(MoveToState com) {
        if (Objects.nonNull(activeMoveCommand) && activeMoveCommand.isScheduled()) {
            activeMoveCommand.cancel();
        }
        activeMoveCommand = com;
    }

    //In class command enable use of private set methods inteded to be abstracted away from normal dev
    public class MoveToState extends CommandBase {
        /** Creates a new SetArmState. */
        private ArmState newState, startState;
        private ArrayList<ArmPose> path;
        private int pathIndex = 0;
        public MoveToState(ArmState targetState) {
            regesterMoveCommand(this);
            setTargetState(targetState);
            path = new ArrayList<>();
            newState = targetState;
            startState = getPrevState();
        }
      
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
            if (newState != ArmState.MANUAL) {
                if (startState != newState) {
                    if (startState != ArmState.CARRY) {
                        if (startState.intermediaryPoses.size() != 0) {
                            ArrayList<ArmPose> ip = startState.getIntermediaryPoses();
                            Collections.reverse(ip);
                            path.addAll(ip);
                        }
                        path.add(ArmState.CARRY.statePose);
                    }
                    if (newState.intermediaryPoses.size() != 0) {
                        path.addAll( newState.getIntermediaryPoses());
                    }
                }
                path.add(newState.statePose);
                setTargetPose(path.get(pathIndex));
            }
         }
      
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            if (atTargetPose() && pathIndex < path.size() - 1 && newState != ArmState.MANUAL) {
                pathIndex++;
                setTargetPose(path.get(pathIndex));
            }
        }
      
        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {}
      
        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
          return atTargetState() || newState == ArmState.MANUAL;
        }
      }
}
