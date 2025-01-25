// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kTopSpeed = 1.0; // 0 to 1
    public static final double kTopAngularSpeed = 1.0; // 0 to 1


    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
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

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
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

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class UnderrollerConstants {
    public static final int kFrontCanId = 21;
    public static final int kRearCanId = 22;
    public static final double kUnderrollerIntakeSpeed = 0.8;
  }

  public static final class HendersonConstants {
    public static final int kLeftLauncherMotorCanId = 1;
    public static final int kRightLauncherMotorCanId = 2;
    public static final int kLeftFeederMotorCanId = 3;
    public static final int kRightFeederMotorCanId = 4;

    public static final double kFeederGearRatio = 47/11;
    public static final double kIntakeLauncherSpeed = 0.1;
    public static final double kIntakeFeederSpeed = 0.3;
    public static double kLauncherIdleRpm = 1000;
    public static double kMaxLauncherRpm = 4000;

    public static double kFeederBackSpeed = 1; // Feeder --> EXIT
    public static double kFeederFrontSpeed = -0.55; // Feeder --> Launcher
    public static double kLauncherFrontSpeed = 0.75; // Launcher --> EXIT
    public static double kLauncherBackSpeed = -0.75; // Launcher --> Feeder
    
    public static double kLauncherIntakeNoteSpeed = -0.75; // Launcher --> Feeder
    public static double kLauncherShootNoteBackSpeed = -0.75; // Launcher --> Feeder
    public static double kLauncherCenteringSpeed = 0.1; // Launcher --> EXIT
  }

  public static final class ArmConstants {
    public static final int kLeftArmMotorCanId = 31;
    public static final int kRightArmMotorCanId = 32;

    public static final double kSportGearRatio = 36.0;
    public static final double kSportPinionTeeth = 10;
    public static final double kArmSprocketTeeth = 36;
    public static final double kArmGearRatio = 129.6;

    // SysID values (in radians and radians/sec)
    public static final double kSVolts = 1.8; //0.11356;
    public static final double kGVolts = 0.5; //0.29175;
    public static final double kVVoltSecondPerRad = 7; //3.65; //1.5928;
    public static final double kAVoltSecondSquaredPerRad = 0.02;//0.030171;

    // Set the arm speed and acceleration
    public static final double kMaxVelocityRadPerSecond = 1.5;
    public static final double kMaxAccelerationRadPerSecSquared = 2;

    public static final double kArmOffsetRads = Units.degreesToRadians(-45); //Starting angle
    public static final double kArmMaxRads = Units.degreesToRadians(90); //Ending angle

    // public static final double kArmEncoderPositionPIDMinInput = kArmOffsetRads; // radians
    // public static final double kArmEncoderPositionPIDMaxInput = kArmMaxRads; 


    public static final double kMaxArmSpeedRpm = 
      NeoMotorConstants.kFreeSpeedRpm / kArmGearRatio ;
    public static final double kMaxArmRadiansPerSecond =
      Units.rotationsPerMinuteToRadiansPerSecond(kMaxArmSpeedRpm);

    public static final double kMaxArmSpeed = 0.8;
    //public static final double kArmSlewRate = 2;
    public static final double kArmDeadband = 0.1;

    public static final double kArmTestOffsetRads = Units.degreesToRadians(15);
    public static final double kArmShootingAngleRads = Units.degreesToRadians(57.5); //Amp shooting agngle
    public static final double kArmFarShootingAngleRads = Units.degreesToRadians(47.5); //Connect to chain
    public static final double kArmPickupAngleRads = Units.degreesToRadians(-39); //37.5 Normal Pickup & Auto Shoot
    //public static final double kArmStraightUpAngleRads = Units.degreesToRadians(90 );
    public static final double kArmDownRads = Units.degreesToRadians(-41); //-40 Jiggle & Trap 
    public static final double kArmShootingStepsRads = (kArmShootingAngleRads - kArmFarShootingAngleRads) / 3; //20
    public static final double kArmPickupStepsRads = Units.degreesToRadians(1); //20
    public static final double kArmFeederAngle = Units.degreesToRadians(-20); //20

  }

}
