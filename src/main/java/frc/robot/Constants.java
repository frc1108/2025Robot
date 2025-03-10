// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.92; //4.92 is maximum 
    public static final double kMaxAngularSpeed = 2*Math.PI; // radians per second //2*Pi is maximum
    public static final double kTopSpeed = 1.0; // 0 to 1
    public static final double kTopAngularSpeed = 1.0; // 0 to 1
    public static final Alliance kAlliance = Alliance.Red;


    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
        

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    // public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = VortexMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.076; //New - .0762 Worn - .07395 Halfway Worn - .075
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  // public static final class AutoConstants {
  //   public static final double kMaxSpeedMetersPerSecond = 3;
  //   public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  //   public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  //   public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

  //   public static final double kPXController = 1;
  //   public static final double kPYController = 1;
  //   public static final double kPThetaController = 1;

  //   // Constraint for the motion profiled robot angle controller
  //   public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
  //       kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  // }

  public static final class VortexMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  public static final class CoralSubsystemConstants {
    public static final int kElevatorMotorCanId = 21;
    public static final int kArmMotorCanId = 41;
    public static final int kIntakeMotorCanId = 42;

    public static final class ElevatorSetpoints {
      public static final double kStow = 0.0;
      public static final double kFeederStation = 0.0; //b intake
      public static final double kLevel1 = 0.0; //povDown
      public static final double kLevel2 = 0.075; //a L2 0.1
      public static final double kLevel3 = 0.45; //x L3 0.475
      public static final double kLevel4 = 0.7; //y L4
    }

    public static final class ArmSetpoints {
      public static final double kStow = 0.3;
      public static final double kFeederStation = 3.34; //b intake
      public static final double kLevel1 = 2.3; //povDown
      public static final double kLevel2 = 3.95; //a L2
      public static final double kLevel3 = 3.9; //x L3
      public static final double kLevel4 = 2.1; //y L4
    }

    public static final class IntakeSetpoints {
      public static final double kForward = -1.0;
      public static final double kReverse = 1.0;
    }
  }

public static final class BargeVisionConstants {
        public static final String kCameraName = "Barge Tag Camera";
        public static final Transform3d kCameraOffset = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6.25), // 5.5 in
                Units.inchesToMeters(-4.25), // -8.75 in
                Units.inchesToMeters(39.375)), //8.5 in
            new Rotation3d(
              Rotation2d.fromDegrees(270).getRadians(), // 
                Rotation2d.fromDegrees(360-37).getRadians(), //360-35
                Rotation2d.fromDegrees(360-5).getRadians() //0
            ));
        public static final double kMaxDistanceMeters = 3;
    }

// Lower camera position constants
// public static final class BargeVisionConstants {
//       public static final String kCameraName = "Barge Tag Camera";
//       public static final Transform3d kCameraOffset = new Transform3d(
//           new Translation3d(
//               Units.inchesToMeters(-5.5), // 5.5 in
//               Units.inchesToMeters(5.5), // -12 in
//               Units.inchesToMeters(7.375)), //8.5 in
//           new Rotation3d(
//               0.0,
//               Rotation2d.fromDegrees(32.0).getRadians(), //22
//               Rotation2d.fromDegrees(3.5).getRadians()
//           ));
//       public static final double kMaxDistanceMeters = 3;
//   }

    public static final class ReefVisionConstants {
      public static final String kCameraName = "Reef Tag Camera";
      public static final Transform3d kCameraOffset = new Transform3d(
          new Translation3d(
              Units.inchesToMeters(6.5), // 1.0 in 6.5
              Units.inchesToMeters(-5.375), // -12 in
              Units.inchesToMeters(15)), //8.5 in
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(11).getRadians(), //22
              Rotation2d.fromDegrees(0).getRadians()
          ));
      public static final double kMaxDistanceMeters = 3;
  }

}
