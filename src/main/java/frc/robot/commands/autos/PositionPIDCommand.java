// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static frc.robot.Constants.AutoConstants.kEndTriggerDebounce;
import static frc.robot.Constants.AutoConstants.kPositionTolerance;
import static frc.robot.Constants.AutoConstants.kRotationTolerance;
import static frc.robot.Constants.AutoConstants.kSpeedTolerance;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class PositionPIDCommand extends Command{
    
    public DriveSubsystem m_swerve;
    public final Pose2d goalPose;
    private PPHolonomicDriveController mDriveController = AutoConstants.kAutoAlignPIDController;

    private final Timer timer = new Timer();

    private final Debouncer endTriggerDebouncer = new Debouncer(kEndTriggerDebounce.in(Seconds));

    private final DoublePublisher xErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("X Error").publish();
    private final DoublePublisher yErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("Y Error").publish();



    private PositionPIDCommand(DriveSubsystem m_swerve, Pose2d goalPose) {
        this.m_swerve = m_swerve;
        this.goalPose = goalPose;
    }

    public static Command generateCommand(DriveSubsystem swerve, Pose2d goalPose, Time timeout){
        return new PositionPIDCommand(swerve, goalPose).withTimeout(timeout).finallyDo(() -> {
            swerve.driveRobotRelative(new ChassisSpeeds(0,0,0));
            swerve.setX();
        });
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        m_swerve.driveRobotRelative(
            mDriveController.calculateRobotRelativeSpeeds(
                m_swerve.getPose(), goalState
            )
        );

        xErrLogger.accept(m_swerve.getPose().getX() - goalPose.getX());
        yErrLogger.accept(m_swerve.getPose().getY() - goalPose.getY());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        Pose2d diff = m_swerve.getPose().relativeTo(goalPose);

        System.out.println("Adjustments to alginment took: " + timer.get() + " seconds and interrupted = " + interrupted
            + "\nPosition offset: " + Centimeter.convertFrom(diff.getTranslation().getNorm(), Meters) + " cm"
            + "\nRotation offset: " + diff.getRotation().getMeasure().in(Degrees) + " deg"
            + "\nVelocity value: " + m_swerve.getSpeed() + "m/s"
        );
    }

    @Override
    public boolean isFinished() {

        Pose2d diff = m_swerve.getPose().relativeTo(goalPose);

        var rotation = MathUtil.isNear(
            0.0, 
            diff.getRotation().getRotations(), 
            kRotationTolerance.getRotations(), 
            0.0, 
            1.0
        );

        var position = diff.getTranslation().getNorm() < kPositionTolerance.in(Meters);

        var speed = m_swerve.getSpeed() < kSpeedTolerance.in(MetersPerSecond);

        // System.out.println("end trigger conditions R: "+ rotation + "\tP: " + position + "\tS: " + speed);
        
        return endTriggerDebouncer.calculate(
            rotation && position && speed
        );
    }
}

