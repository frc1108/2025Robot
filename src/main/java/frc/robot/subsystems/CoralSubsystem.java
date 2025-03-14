// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralSubsystemConstants;
import frc.robot.Constants.CoralSubsystemConstants.ArmSetpoints;
import frc.robot.Constants.CoralSubsystemConstants.ElevatorSetpoints;


@Logged
public class CoralSubsystem extends SubsystemBase {
  public enum Setpoint {
    kStow,
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
  }

  // Initialize arm SPARK.  Using MaxMotion with Absolute REV Through Bore for control.  So initialize
  // the closed loop controller and encoder
  private final SparkMax m_armMotor = new SparkMax(CoralSubsystemConstants.kArmMotorCanId, MotorType.kBrushless);
  private final SparkClosedLoopController m_armController = m_armMotor.getClosedLoopController();
  private final AbsoluteEncoder m_armEncoder = m_armMotor.getAbsoluteEncoder();

  // Initialize elevator SPARK.  Using MaxMotion with Relative REV Through Bore for control.  So initialize
  // the closed loop controller and encoder
  private final SparkMax m_elevatorMotor = new SparkMax(CoralSubsystemConstants.kElevatorMotorCanId, MotorType.kBrushless);
  private final SparkClosedLoopController m_elevatorController = m_elevatorMotor.getClosedLoopController();
  private final RelativeEncoder m_elevatorEncoder = m_elevatorMotor.getAlternateEncoder();
  private final DigitalInput m_elevatorLowerLimit = new DigitalInput(0);


  // Managing the state of the robot
  private boolean m_resetByButton = false;
  private boolean m_resetByLimit = false;
  private double m_armCurrentTarget = ArmSetpoints.kStow;
  private double m_elevatorCurrentTarget = ElevatorSetpoints.kStow;

  /** Creates a new Elevator. */
  public CoralSubsystem() {
    m_armMotor.configure(
      Configs.Arm.armConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    m_elevatorMotor.configure(
      Configs.Elevator.elevatorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);


      // Zero arm and elevator encoders on initialization
      //m_armEncoder.setPosition(0); //Only for Relative encoder, using Absolute.
      m_elevatorEncoder.setPosition(0);
  }

  /**
   * Drive the arm and elevator motors to their setpoints.  This uses MAXMotion
   * position controls for smooth trapezoidal motions
   */
  private void moveToSetpoint() {
    m_armController.setReference(m_armCurrentTarget, ControlType.kMAXMotionPositionControl);
    m_elevatorController.setReference(
      m_elevatorCurrentTarget,ControlType.kMAXMotionPositionControl);
  }

  private boolean lowerLimitIsPressed () {
    return m_elevatorLowerLimit.get();
  }

  //** Zero the elevator encoder when limit switch is pressed */
  private void zeroElevatorOnLimitSwitch() {
    if (!m_resetByLimit && lowerLimitIsPressed()) {
      // Zero the encoder only when the lmit switch is not already reset
      m_elevatorEncoder.setPosition(0);
      m_resetByLimit = true;
    } else if (!lowerLimitIsPressed()) {
      m_resetByLimit = false;
    }
  }

private void zeroOnUserButton() {
  if (!m_resetByButton && RobotController.getUserButton()) {
    m_resetByButton = true;
    m_elevatorEncoder.setPosition(0);
  } else if (!RobotController.getUserButton()) {
    m_resetByButton = false;
  }
}



/** 
 * Command to set the subsystem setpoint. This will set to predefined positions for the given setpoints
 */
public Command setSetpointCommand(Setpoint setpoint) {
  return this.runOnce(
    () -> {
      switch (setpoint) {
        case kStow:
          m_armCurrentTarget = ArmSetpoints.kStow;
          m_elevatorCurrentTarget = ElevatorSetpoints.kStow;
          break;
        case kFeederStation:
          m_armCurrentTarget = ArmSetpoints.kFeederStation;
          m_elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
          break;
        case kLevel1:
          m_armCurrentTarget = ArmSetpoints.kLevel1;
          m_elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
          break;
        case kLevel2:
          m_armCurrentTarget = ArmSetpoints.kLevel2;
          m_elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
          break;
        case kLevel3:
          m_armCurrentTarget = ArmSetpoints.kLevel3;
          m_elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
          break;
        case kLevel4:
          m_armCurrentTarget = ArmSetpoints.kLevel4;
          m_elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
          break;
      }
    });
}

  @Override
  public void periodic() {
    moveToSetpoint();
    zeroElevatorOnLimitSwitch();
    zeroOnUserButton();
    // This method will be called once per scheduler run
  }

  public double getElevatorAlternateEncoder() {
    return m_elevatorEncoder.getPosition();
  }

  public double getArmAbsoluteEncoder() {
    return m_armEncoder.getPosition();
  }

}
