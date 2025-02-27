// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoralSubsystemConstants.ArmSetpoints;
import frc.robot.Constants.CoralSubsystemConstants.ElevatorSetpoints;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
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
  private final AlgaeSubsystem m_algae = new AlgaeSubsystem();
  private final CoralSubsystem m_coral = new CoralSubsystem();

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
    //configureEventTriggers();
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser",m_autoChooser);
    setupPathPlannerLog();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                m_invertDriveAlliance*MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                m_invertDriveAlliance*MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
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
    m_driverController.leftTrigger().whileFalse(m_climber.stopClimber());
    m_driverController.rightTrigger().whileFalse(m_climber.stopClimber());

    m_operatorController.rightBumper().whileTrue(m_algae.upAlgae());
    m_operatorController.leftBumper().whileTrue(m_algae.downAlgae());
    m_operatorController.leftBumper().whileFalse(m_algae.stopAlgae());
    m_operatorController.rightBumper().whileFalse(m_algae.stopAlgae());

    m_operatorController.povLeft().whileTrue(m_algae.inAlgaeRoller());
    m_operatorController.povRight().whileTrue(m_algae.outAlgaeRoller());
    m_operatorController.povLeft().whileFalse(m_algae.stopAlgaeRoller());
    m_operatorController.povRight().whileFalse(m_algae.stopAlgaeRoller());

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
    m_operatorController.leftTrigger().whileTrue(m_coral.runIntakeCommand()); //In
    m_operatorController.rightTrigger().whileTrue(m_coral.reverseIntakeCommand()); //OutOperator
    m_driverController.rightBumper().whileTrue(m_coral.reverseIntakeCommand()); //OutDriver
    m_operatorController.povUp().whileTrue(m_coral.reverseSlowIntakeCommand()); //Slow Out
    m_operatorController.start().whileTrue(m_coral.reverseFastIntakeCommand()); //Fast Out
    //Coral/Elevator Positions
    m_operatorController.b().onTrue(m_coral.setSetpointCommand(Setpoint.kFeederStation)); //Pickup
    m_operatorController.a().onTrue(m_coral.setSetpointCommand(Setpoint.kLevel2)); //L2
    m_operatorController.x().onTrue(m_coral.setSetpointCommand(Setpoint.kLevel3)); //L3
    m_operatorController.y().onTrue(m_coral.setSetpointCommand(Setpoint.kLevel4)); //L4
    m_operatorController.povDown().onTrue(m_coral.setSetpointCommand(Setpoint.kLevel1)); //L4Down
    m_operatorController.back().onTrue(m_coral.setSetpointCommand(Setpoint.kStow)); //Stow
    
m_driverController.a()
    .onTrue(new InstantCommand(() -> {
        m_operatorController.setRumble(RumbleType.kLeftRumble, 1);
        m_operatorController.setRumble(RumbleType.kRightRumble, 1); // Set both rumble types on True
    }))
    .onFalse(new InstantCommand(() -> {
        m_operatorController.setRumble(RumbleType.kLeftRumble, 0);
        m_operatorController.setRumble(RumbleType.kRightRumble, 0); // Set both rumble types on False
    }));




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
    NamedCommands.registerCommand("intakePosition", m_coral.setSetpointCommand(Setpoint.kFeederStation));
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
    return
      Commands.parallel(
        m_coral.runIntakeCommand(),
        Commands.waitSeconds(2)
      );
  }
  public Command reverseCoral() {
    return 
      Commands.parallel(
        m_coral.reverseIntakeCommand(),
        Commands.waitSeconds(2)
        );
  }
  public Command reverseSlowCoral() {
    return 
      Commands.parallel(
        m_coral.reverseSlowIntakeCommand(),
        Commands.waitSeconds(.25)
        );
  }
  public Command intakeAlgae() {
    return 
      Commands.parallel(
        m_algae.inAlgaeRoller(),
        Commands.waitSeconds(1)
        );
  }
  public Command reverseIntakeAlgae() {
    return 
      Commands.parallel(
        m_algae.outAlgaeRoller(),
        Commands.waitSeconds(1)
        );
  }
  public Command upAlgae() {
    return 
      Commands.parallel(
        m_algae.upAlgae(),
        Commands.waitSeconds(1)
        );
  }
  public Command downAlgae() {
    return 
      Commands.parallel(
        m_algae.downAlgae(),
        Commands.waitSeconds(1)
        );
  }
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
}
