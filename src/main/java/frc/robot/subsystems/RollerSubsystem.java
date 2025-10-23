// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.RollerConstants;


@Logged
public class RollerSubsystem extends SubsystemBase {

  final SparkMax m_algaeRoller = new SparkMax(RollerConstants.kAlgaeRollerCanId, MotorType.kBrushless);
  final SparkMax m_coralL1Roller = new SparkMax(RollerConstants.kCoralRollerCanId, MotorType.kBrushless);
  final MedianFilter m_currentFilter = new MedianFilter(100);

  /** Creates a new AlgaeSubsystem. */
  public RollerSubsystem() {
    m_algaeRoller.configure(
      Configs.AlgaeIntake.algaeIntakeConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    m_coralL1Roller.configure(
      Configs.CoralIntake.coralIntakeConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {}
  
  private void setSpinPower(double power) {
    m_algaeRoller.set(power);
  }

  public Command inAlgaeRoller() {
    return this.startEnd(
      () -> {
        this.setSpinPower(0.5);
        this.setCoralSpinPower(0.5);
      },
      () -> {
        this.setSpinPower(0.0);
        this.setCoralSpinPower(0.0);
      });
  }

  public Command outAlgaeRoller() {
    return this.startEnd(
      () -> {
        this.setSpinPower(-0.5);
        this.setCoralSpinPower(-0.5);
      },
      () -> {
        this.setSpinPower(0.0);
        this.setCoralSpinPower(0.0);
      });
  }

  private void setCoralSpinPower(double power) {
    m_coralL1Roller.set(power);
  }

  public Command inCoralRoller() {
    return this.startEnd(
      () -> {
        this.setSpinPower(0.5);
        this.setCoralSpinPower(-0.5);
      },
      () -> {
        this.setSpinPower(0.0);
        this.setCoralSpinPower(0.0);
      });
  }

  public Command outCoralRollerSlow() {
    return this.startEnd(
      () -> {
        this.setSpinPower(-0.4);
        this.setCoralSpinPower(0.1);
      },
      () -> {
        this.setSpinPower(0.0);
        this.setCoralSpinPower(0.0);
      });
  }

  public Command outCoralRollerFast() {
    return this.startEnd(
      () -> {
        this.setSpinPower(-0.5);
        this.setCoralSpinPower(0.5);
      },
      () -> {
        this.setSpinPower(0.0);
        this.setCoralSpinPower(0.0);
      });
  }

  public double getCoralIntakeCurrent() {
    return m_coralL1Roller.getOutputCurrent();
  }

  public double getFilteredCoralIntakeCurrent() {
    return m_currentFilter.calculate(getCoralIntakeCurrent());
  }

  public boolean isCoralPresent() {
    return m_coralL1Roller.getForwardLimitSwitch().isPressed();
  }
  // public boolean isCoralPresent() {
  //   var coralStallCurrent = 10.0;
  //   return (getFilteredCoralIntakeCurrent() > coralStallCurrent);
  // }
}