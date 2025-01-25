// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.DoubleSupplier;


import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;

@Logged
public class Arm extends TrapezoidProfileSubsystem{
  /** Creates a new Arm. */
  private final SparkMax m_leftMotor = new SparkMax(ArmConstants.kLeftArmMotorCanId, MotorType.kBrushless);
  private final SparkMax m_rightMotor = new SparkMax(ArmConstants.kRightArmMotorCanId, MotorType.kBrushless);

  private final SparkClosedLoopController m_pid;
  private final RelativeEncoder m_encoder;

  private final ArmFeedforward m_feedforward = 
  new ArmFeedforward(
    ArmConstants.kSVolts, ArmConstants.kGVolts, 
    ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);
  private double m_goal = ArmConstants.kArmOffsetRads;
 
  public Arm() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(
          ArmConstants.kMaxVelocityRadPerSecond, 
          ArmConstants.kMaxAccelerationRadPerSecSquared),
        // The initial position of the mechanism
        ArmConstants.kArmOffsetRads);

  m_leftMotor.configure(Configs.Arm.armConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
  m_rightMotor.configure(new SparkMaxConfig()
              .follow(m_leftMotor,true),ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

  m_encoder = m_leftMotor.getEncoder();
  m_encoder.setPosition(ArmConstants.kArmOffsetRads);

  m_pid = m_leftMotor.getClosedLoopController();
  }

  @Override
  public void periodic(){
    super.setGoal(m_goal);
    super.periodic();
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
  // Calculate the feedforward from the sepoint
  double feedforward = m_feedforward.calculate(setpoint.position,
                                               setpoint.velocity);


  // Add the feedforward to the PID output to get the motor outputF
  m_pid.setReference(setpoint.position, // - ArmConstants.kArmOffsetRads,
                      ControlType.kPosition,ClosedLoopSlot.kSlot0, feedforward);
  }

  public Command setArmGoalCommand(double goal) {
  return Commands.runOnce(() -> setArmGoal(goal), this);
  }

  public void set(double speed) {
    m_leftMotor.set(speed);
  }

  @Logged(name = "Arm Pos Rads")
  public double getPositionRadians() {
    return m_encoder.getPosition();
  }

  @Logged(name = "Arm Goal Rads")
  public double getArmGoal() {
    return m_goal;
  }

  public void setArmGoal(double goal) {
      m_goal = MathUtil.clamp(goal,ArmConstants.kArmOffsetRads-0.1,ArmConstants.kArmMaxRads+0.1);
  }

  public Command setArmManual(DoubleSupplier speed) {
    return Commands.run(()->setArmGoal(getArmGoal()+speed.getAsDouble()/(2*Math.PI)),this);
  }

  public void setEncoderPosition(double position) {
    m_encoder.setPosition(position);
  }

  @Logged(name = "Arm, Amps")
  public double getMotorCurrent(){
    return m_leftMotor.getOutputCurrent();
  }

//   public void setIdle(IdleMode mode) {
//     m_leftMotor.setIdleMode(mode);
//   }

}
  
