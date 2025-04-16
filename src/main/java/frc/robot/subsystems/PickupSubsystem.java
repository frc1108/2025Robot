// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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
import frc.robot.Constants.PickupSubsystemConstants.PickupSetpoints;
import frc.robot.Constants.PickupSubsystemConstants;


@Logged
public class PickupSubsystem extends SubsystemBase {
  public enum PickupSetpoint {
    kStow,
    kAlgaeStowL1,
    kAlgaePickup,
    kCoralPickup,
    kMax
  }

  // Initialize arm SPARK.  Using MaxMotion with Absolute REV Through Bore for control.  So initialize
  // the closed loop controller and encoder
  private final SparkMax m_pickupMotor = new SparkMax(PickupSubsystemConstants.kPickupMotorCanId, MotorType.kBrushless);
  private final SparkClosedLoopController m_pickupController = m_pickupMotor.getClosedLoopController();
  private final RelativeEncoder m_pickupEncoder = m_pickupMotor.getEncoder();

  // Managing the state of the robot
  private boolean m_resetByButton = false;
  private boolean m_resetByLimit = false;
  private double m_pickupCurrentTarget = PickupSetpoints.kStow;

  /** Creates a new Elevator. */
  public PickupSubsystem() {
    m_pickupMotor.configure(
      Configs.Pickup.pickupConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

      // Zero arm and elevator encoders on initialization
      //m_armEncoder.setPosition(0); //Only for Relative encoder, using Absolute.
      m_pickupEncoder.setPosition(0);
  }

  /**
   * Drive the arm and elevator motors to their setpoints.  This uses MAXMotion
   * position controls for smooth trapezoidal motions
   */
  private void moveToSetpoint() {
    m_pickupController.setReference(m_pickupCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  // private boolean lowerLimitIsPressed () {
  //   return m_elevatorLowerLimit.get();
  // }

  // //** Zero the elevator encoder when limit switch is pressed */
  // private void zeroElevatorOnLimitSwitch() {
  //   if (!m_resetByLimit && lowerLimitIsPressed()) {
  //     // Zero the encoder only when the lmit switch is not already reset
  //     m_elevatorEncoder.setPosition(0);
  //     m_resetByLimit = true;
  //   } else if (!lowerLimitIsPressed()) {
  //     m_resetByLimit = false;
  //   }
  // }

private void zeroOnUserButton() {
  if (!m_resetByButton && RobotController.getUserButton()) {
    m_resetByButton = true;
    m_pickupEncoder.setPosition(0);
  } else if (!RobotController.getUserButton()) {
    m_resetByButton = false;
  }
}



/** 
 * Command to set the subsystem setpoint. This will set to predefined positions for the given setpoints
 */
public Command setSetpointCommand(PickupSetpoint setpoint) {
  return this.runOnce(
    () -> {
      switch (setpoint) {
        case kStow:
          m_pickupCurrentTarget = PickupSetpoints.kStow;
          break;
        case kAlgaeStowL1:
          m_pickupCurrentTarget = PickupSetpoints.kAlgaeStowL1;
          break;
        case kAlgaePickup:
          m_pickupCurrentTarget = PickupSetpoints.kAlgaePickup;
          break;
        case kCoralPickup:
          m_pickupCurrentTarget = PickupSetpoints.kCoralPickup;
          break;
        case kMax:
          m_pickupCurrentTarget = PickupSetpoints.kMax;
          break;
      }
    });
}

  @Override
  public void periodic() {
    moveToSetpoint();
    //zeroElevatorOnLimitSwitch();
    zeroOnUserButton();
    // This method will be called once per scheduler run
  }

  public double getPickupEncoder() {
    return m_pickupEncoder.getPosition();
  }
}
