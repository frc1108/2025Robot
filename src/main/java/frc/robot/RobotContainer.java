// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.PickupSubsystem.PickupSetpoint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final RollerSubsystem m_roller = new RollerSubsystem();
  private final CoralSubsystem m_coral = new CoralSubsystem();
  private final PickupSubsystem m_pickup = new PickupSubsystem();
  private Vision m_reefVision, m_twoReefVision, m_bargeVision;
  private final LEDSubsystem m_led = new LEDSubsystem();
  private final CoralIntakeSubsystem m_coralIntake = new CoralIntakeSubsystem();


  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final Field2d m_path;
  private final SendableChooser<Command> m_autoChooser;
  private int m_invertDriveAlliance = -1;  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_path = new Field2d();

    // Configure the button bindings
    configureButtonBindings();
    configureNamedCommands();
    //configureWithAlliance(Constants.DriveConstants.kAlliance);
    //configureEventTriggers();
    //m_autoChooser = AutoBuilder.buildAutoChooser();
    m_autoChooser = new SendableChooser<>();

    //Elevator towards reef side (ER)
    m_autoChooser.addOption("ER 3Three Coral Right B3", this.ThreeCoralRight());
    m_autoChooser.addOption("Three Coral Left B4", this.ThreeCoralLeft());

    // Elevator towards barge (EB)
    m_autoChooser.addOption("2 Coral B1", AutoBuilder.buildAuto("2C B1C12L4,C12S2,S2C1L4"));
    m_autoChooser.addOption("3 Coral B2", AutoBuilder.buildAuto("T3C B2C12L4,C12S2,S2C1L4"));
    m_autoChooser.addOption("3 Coral BX", AutoBuilder.buildAuto("TTwo2C BXC12L4,C12S2,S2C1L4"));

    //m_autoChooser.addOption("T2C BBCC10L4, C10S1, S1C11L4", AutoBuilder.buildAuto("T2C BBCC10L4, C10S1, S1C11L4"));
    m_autoChooser.addOption("LeftTwo_9-8_S6", AutoBuilder.buildAuto("LeftTwo_9-8_S6"));
    m_autoChooser.addOption("RightTwo_10-11_S1", AutoBuilder.buildAuto("RightTwo_10-11_S1"));
    m_autoChooser.addOption("3 Coral B6", AutoBuilder.buildAuto("2C B6C7L4,C7S5,S5C6L4"));
    m_autoChooser.addOption("Left_3_7-5_S5", AutoBuilder.buildAuto("Left_3_7-5_S5"));

    // Elevator angled towards reef (EATR)

    //Test autos
    m_autoChooser.addOption("TEST 12 Ft", AutoBuilder.buildAuto("12 Ft"));
    //m_autoChooser.addOption("RightTwo_10-11_S1", AutoBuilder.buildAuto("RightTwo_10-11_S1"));
    SmartDashboard.putData("Auto Chooser",m_autoChooser);
    setupPathPlannerLog();
    try {
      m_reefVision = new Vision(m_robotDrive::visionPose, m_robotDrive,Constants.ReefVisionConstants.kCameraName,Constants.ReefVisionConstants.kCameraOffset);
      m_twoReefVision = new Vision(m_robotDrive::visionPose, m_robotDrive,Constants.TwoReefVisionConstants.kCameraName,Constants.TwoReefVisionConstants.kCameraOffset);
      m_bargeVision= new Vision(m_robotDrive::visionPose, m_robotDrive,Constants.BargeVisionConstants.kCameraName,Constants.BargeVisionConstants.kCameraOffset);
    }
     catch(IOException e) {
     DriverStation.reportWarning("Unable to initialize vision", e.getStackTrace());
    }

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                m_invertDriveAlliance*MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                m_invertDriveAlliance*MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                // m_robotDrive.isAllianceFlipped()?1:-1*MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                // m_robotDrive.isAllianceFlipped()?1:-1*MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    m_driverController.rightTrigger().whileTrue(m_climber.upClimber());
    m_driverController.leftTrigger().whileTrue(m_climber.downClimber());

    // m_operatorController.rightBumper().whileTrue(m_algae.upAlgae());
    // m_operatorController.leftBumper().whileTrue(m_algae.downAlgae());

    m_operatorController.povLeft().whileTrue(m_roller.inAlgaeRoller());
    m_operatorController.povRight().whileTrue(m_roller.outAlgaeRoller());

    m_operatorController.povLeft().and(m_operatorController.rightBumper()).whileTrue(m_roller.inAlgaeRoller());
    m_operatorController.povRight().and(m_operatorController.leftBumper()).whileTrue(m_roller.outAlgaeRoller());


    // m_operatorController.rightTrigger().whileTrue(m_coral.coralSpinIn());
    // m_operatorController.leftTrigger().whileTrue(m_coral.coralSpinOut());
    // m_operatorController.rightTrigger().whileFalse(m_coral.coralSpinStop());
    // m_operatorController.leftTrigger().whileFalse(m_coral.coralSpinStop());

    // m_operatorController.a().whileTrue(m_coral.coralIn());
    // m_operatorController.b().whileTrue(m_coral.coralOut());
    // m_operatorController.a().whileFalse(m_coral.coralStop());
    // m_operatorController.b().whileFalse(m_coral.coralStop());

    // m_operatorController.povDown().whileTrue(m_coral.elevatorUp());
    // m_operatorController.povUp().whileTrue(m_coral.elevatorDown());
    // m_operatorController.povDown().whileFalse(m_coral.elevatorStop());
    // m_operatorController.povUp().whileFalse(m_coral.elevatorStop());

    
    // new JoystickButton(m_driverController, Button.kR1.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));

    //Coral Intake
    m_operatorController.leftTrigger().whileTrue(m_coralIntake.runIntakeCommand()); //In
    m_operatorController.rightTrigger().whileTrue(m_coralIntake.reverseIntakeCommand()); //OutOperator
    m_driverController.rightBumper().whileTrue(m_coralIntake.reverseIntakeCommand()); //OutDriver
    m_operatorController.povUp().whileTrue(m_coralIntake.reverseSlowIntakeCommand()); //Slow Out
    m_operatorController.start().whileTrue(m_coralIntake.reverseFastIntakeCommand()); //Fast Out
    //Coral/Elevator Positions
    m_operatorController.b().onTrue(m_coral.setSetpointCommand(Setpoint.kFeederStation)); //Pickup
    m_operatorController.a().onTrue(m_coral.setSetpointCommand(Setpoint.kLevel2)); //L2
    m_operatorController.x().onTrue(m_coral.setSetpointCommand(Setpoint.kLevel3)); //L3
    m_operatorController.y().onTrue(m_coral.setSetpointCommand(Setpoint.kLevel4)); //L4
    m_operatorController.povDown().onTrue(m_coral.setSetpointCommand(Setpoint.kLevel1)); //L4Down
    m_operatorController.back().onTrue(m_coral.setSetpointCommand(Setpoint.kStow)); //Stow

    var joystickValueForAxisButton = 0.75;

    m_operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value,joystickValueForAxisButton).onTrue(this.none());  // Left stick up
    m_operatorController.axisLessThan(XboxController.Axis.kLeftY.value,-joystickValueForAxisButton).onTrue(this.none());  // Left stick up
    m_operatorController.leftStick().whileTrue(m_roller.outCoralRollerFast());
    
    m_operatorController.axisGreaterThan(XboxController.Axis.kLeftX.value,joystickValueForAxisButton).whileTrue(m_roller.outCoralRollerSlow());  // Left stick right
    m_operatorController.axisLessThan(XboxController.Axis.kLeftX.value,-joystickValueForAxisButton)
      .whileTrue(m_roller.inCoralRoller());  // Left stick left
    
    m_operatorController.axisGreaterThan(XboxController.Axis.kRightY.value,joystickValueForAxisButton)
      .onTrue(m_pickup.setSetpointCommand(PickupSetpoint.kCoralPickup));  // Left stick up
    m_operatorController.axisLessThan(XboxController.Axis.kRightY.value,-joystickValueForAxisButton)
      .onTrue(m_pickup.setSetpointCommand(PickupSetpoint.kAlgaeStowL1));  // Left stick up
    
    m_operatorController.axisGreaterThan(XboxController.Axis.kRightX.value,joystickValueForAxisButton)
      .onTrue(m_pickup.setSetpointCommand(PickupSetpoint.kAlgaePickup));  // Left stick up
    m_operatorController.axisLessThan(XboxController.Axis.kRightX.value,-joystickValueForAxisButton)
      .onTrue(m_pickup.setSetpointCommand(PickupSetpoint.kStow));  // Left stick up
    m_operatorController.rightStick()
      .onTrue(m_pickup.setSetpointCommand(PickupSetpoint.kMax));
    

m_driverController.a()
    .onTrue(new InstantCommand(() -> {
        m_operatorController.setRumble(RumbleType.kLeftRumble, 1);
        m_operatorController.setRumble(RumbleType.kRightRumble, 1); // Set both rumble types on True
    }))
    .onFalse(new InstantCommand(() -> {
        m_operatorController.setRumble(RumbleType.kLeftRumble, 0);
        m_operatorController.setRumble(RumbleType.kRightRumble, 0); // Set both rumble types on False
    }));

m_driverController.x().onTrue(m_led.runPattern(LEDPattern.solid(Color.kWhite)));
m_driverController.b().onTrue(m_led.runPattern(LEDPattern.solid(Color.kBlue)));

m_driverController.y().onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));
m_driverController.povUp().onTrue(this.intakeCoral());

new Trigger(()->m_coralIntake.isCoralPresent())
                .onTrue(m_led.runPattern(LEDPattern.solid(Color.kWhite)))
                .onFalse(m_led.runPattern(LEDPattern.solid(Color.kBlack)));
 // RobotModeTriggers.disabled().onTrue(m_coral.setSetpointCommand(Setpoint.kLevel1));




    //m_driverController.start().onTrue(m_robotDrive.zeroHeading();

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
  // Commands for Autos
  public void configureNamedCommands() {
    NamedCommands.registerCommand("levelFourUp", m_coral.setSetpointCommand(Setpoint.kLevel4));
    NamedCommands.registerCommand("levelThreeUp", m_coral.setSetpointCommand(Setpoint.kLevel3));
    NamedCommands.registerCommand("levelTwoUp", m_coral.setSetpointCommand(Setpoint.kLevel2));
    NamedCommands.registerCommand("down", m_coral.setSetpointCommand(Setpoint.kLevel1));
    NamedCommands.registerCommand("feederPosition", m_coral.setSetpointCommand(Setpoint.kFeederStation));
    NamedCommands.registerCommand("intakeCoral", intakeCoral());
    NamedCommands.registerCommand("reverseCoral", reverseCoral());
    NamedCommands.registerCommand("reverseSlowCoral", reverseSlowCoral());
    NamedCommands.registerCommand("intakeAlgae", intakeAlgae());
    NamedCommands.registerCommand("reverseIntakeAlgae", reverseIntakeAlgae());
    NamedCommands.registerCommand("upAlgae", upAlgae());
    NamedCommands.registerCommand("downAlgae", downAlgae());
    NamedCommands.registerCommand("none", none());
  }
  /*
  public void configureEventTriggers() {
    new EventTrigger("levelFourUp").onTrue(m_coral.setSetpointCommand(Setpoint.kLevel4));
    new EventTrigger("levelThreeUp").onTrue(m_coral.setSetpointCommand(Setpoint.kLevel3));
    new EventTrigger("levelTwoUp").onTrue(m_coral.setSetpointCommand(Setpoint.kLevel2));
    new EventTrigger("down").onTrue(m_coral.setSetpointCommand(Setpoint.kLevel1));
    new EventTrigger("intakePosition").onTrue(m_coral.setSetpointCommand(Setpoint.kFeederStation));
  }
  */
  //Command Names and Their Actions
  public Command intakeCoral() {
    return Commands.parallel(
        m_coralIntake.runIntakeCommand().withTimeout(2)
    ).until(()->m_coralIntake.isCoralPresent());
}
  public Command reverseCoral() {
    return 
      Commands.parallel(
        m_coralIntake.reverseIntakeCommand().withTimeout(1)
        );
  }
  public Command reverseSlowCoral() {
    return 
      Commands.parallel(
        m_coralIntake.reverseSlowIntakeCommand().withTimeout(.25)
        );
  }
  public Command intakeAlgae() {
    return 
      Commands.parallel(
        m_roller.inAlgaeRoller().withTimeout(1)
        );
  }
  public Command reverseIntakeAlgae() {
    return 
      Commands.parallel(
        m_roller.outAlgaeRoller().withTimeout(1)
        );
  }
  public Command upAlgae() {
    return m_pickup.setSetpointCommand(PickupSetpoint.kAlgaeStowL1);
  }

  // public Command upAlgae() {
  //   return 
  //     Commands.parallel(
  //       m_algae.upAlgae().withTimeout(1)
  //       );
  // }


  public Command downAlgae() {
    return m_pickup.setSetpointCommand(PickupSetpoint.kCoralPickup);
  }

  // public Command downAlgae() {
  //   return 
  //     Commands.parallel(
  //       m_algae.downAlgae().withTimeout(1)
  //       );
  // }
  // public Command downLOneAlgae() {
  //   return 
  //     Commands.parallel(
  //       m_algae.downAlgae().withTimeout(.1)
  //       );
  // }
  public Command none() {
    return 
      Commands.parallel();
  }

  public void configureWithAlliance(Alliance alliance) {
    m_invertDriveAlliance = (alliance == Alliance.Blue)?-1:1;
  }

  private void setupPathPlannerLog() {
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      m_path.setRobotPose(pose);
    });

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      m_path.getObject("target pose").setPose(pose);
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      m_path.getObject("path").setPoses(poses);
    });
  }

  // Command Right3Coral(){
  //   return Commands.sequence(
  //            new PathPlannerAuto("Score Pos 1"),
  //            Commands.either(new PathPlannerAuto("RescoreCoralAtPos1"),
  //                            new PathPlannerAuto("GetCoralFromPos1"),
  //                            m_coralIntake::isCoralPresent),
  //            Commands.either(new PathPlannerAuto("ScorePos2"),
  //                            new PathPlannerAuto("RegetToScorePos2"),
  //                            m_coralIntake::isCoralPresent),
  //            Commands.either(new PathPlannerAuto("RescoreCoralAtPos2"),
  //                            new PathPlannerAuto("GetCoralFromPos2"),
  //                            m_coralIntake::isCoralPresent),
  //            Commands.either(new PathPlannerAuto("ScorePos3"),
  //                            new PathPlannerAuto("RegetToScorePos3"),
  //                            m_coralIntake::isCoralPresent));
  // }

  Command ThreeCoralRight(){
    var reefScoringTimeDelay = 0.3;  //Adjust to reduce stopping time after scoring in seconds
    var coralStationTimeDelay = 0.05;
    BooleanSupplier isAllianceFlipped = () -> m_robotDrive.isAllianceFlipped(); // Supplier so checked at time Command runs
    var isPathMirrored = isAllianceFlipped.getAsBoolean();
    return Commands.sequence(
             new PathPlannerAuto("TAAB3C12",isPathMirrored),
             Commands.waitSeconds(reefScoringTimeDelay),
             Commands.either(new PathPlannerAuto("TAAC12RS1",isPathMirrored),
                             new PathPlannerAuto("TAAC12S1",isPathMirrored),
                             m_coralIntake::isCoralPresent),
             new PathPlannerAuto("TAAS1C1",isPathMirrored),
            //  Commands.waitSeconds(coralStationTimeDelay),
            //  Commands.either(new PathPlannerAuto("TAAS1C1",isPathMirrored),
            //                  new PathPlannerAuto("TAAS1RC1",isPathMirrored),
            //                  m_coralIntake::isCoralPresent),
             Commands.waitSeconds(reefScoringTimeDelay),
             Commands.either(new PathPlannerAuto("TAAC1RS1",isPathMirrored),
                             new PathPlannerAuto("TAAC1S1",isPathMirrored),
                             m_coralIntake::isCoralPresent),
             new PathPlannerAuto("TAAS1C2",isPathMirrored),
            //  Commands.waitSeconds(coralStationTimeDelay),
            //  Commands.either(new PathPlannerAuto("TAAS1C2",isPathMirrored),
            //                  new PathPlannerAuto("TAAS1RC2",isPathMirrored),
            //                  m_coralIntake::isCoralPresent),
             Commands.waitSeconds(reefScoringTimeDelay),
             Commands.either(new PathPlannerAuto("TAAC2RS2",isPathMirrored),
                             new PathPlannerAuto("TAAC2S2",isPathMirrored),
                             m_coralIntake::isCoralPresent));
  }

  Command ThreeCoralLeft(){
    var reefScoringTimeDelay = 0.3;  //Adjust to reduce stopping time after scoring in seconds
    BooleanSupplier isAllianceFlipped = () -> m_robotDrive.isAllianceFlipped(); // Supplier so checked at time Command runs
    var isPathMirrored = isAllianceFlipped.getAsBoolean();
    return Commands.sequence(
             new PathPlannerAuto("TAAB4C7",isPathMirrored),
             Commands.waitSeconds(reefScoringTimeDelay),
             Commands.either(new PathPlannerAuto("TAAC7RS6",isPathMirrored),
                             new PathPlannerAuto("TAAC7S6",isPathMirrored),
                             m_coralIntake::isCoralPresent),
             new PathPlannerAuto("TAAS6C6",isPathMirrored),
             Commands.waitSeconds(reefScoringTimeDelay),
             Commands.either(new PathPlannerAuto("TAAC6RS6",isPathMirrored),
                             new PathPlannerAuto("TAAC6S6",isPathMirrored),
                             m_coralIntake::isCoralPresent),

            //  Commands.either(new PathPlannerAuto("TAAS6C6",isPathMirrored),
            //                  new PathPlannerAuto("TAAS6RC6",isPathMirrored),
            //                  m_coralIntake::isCoralPresent),
            new PathPlannerAuto("TAAS6C6",isPathMirrored),
            Commands.waitSeconds(reefScoringTimeDelay)
             );
  }

  //            Commands.either(new PathPlannerAuto("TAAC1RS1",isPathMirrored),
  //                            new PathPlannerAuto("TAAC1S1",isPathMirrored),
  //                            m_coralIntake::isCoralPresent),
  //           Commands.either(new PathPlannerAuto("TAAS1C2",isPathMirrored),
  //                            new PathPlannerAuto("TAAS1RC2",isPathMirrored),
  //                            m_coralIntake::isCoralPresent),
  //           Commands.waitSeconds(reefScoringTimeDelay),
  //           Commands.either(new PathPlannerAuto("TAAC2RS2",isPathMirrored),
  //                            new PathPlannerAuto("TAAC2S2",isPathMirrored),
  //                            m_coralIntake::isCoralPresent));
  // }


}
