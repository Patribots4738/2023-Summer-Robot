// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain extends SubsystemBase{
  /** Creates a new Drivetrain. */
  private final CANSparkMax leftLeadMotor;
  private final CANSparkMax rightLeadMotor;
  private final CANSparkMax leftFollower;
  private final CANSparkMax rightFollower;

  private final DifferentialDrive m_drive;

  public Drivetrain() {

    // initialize motors
    leftLeadMotor = new CANSparkMax(Constants.LEFT_MOTOR_FRONT_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightLeadMotor = new CANSparkMax(Constants.RIGHT_MOTOR_FRONT_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftFollower = new CANSparkMax(Constants.LEFT_MOTOR_FOLLOWER_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightFollower = new CANSparkMax(Constants.RIGHT_MOTOR_FOLLOWER_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    
    // follow the lead motors
    leftFollower.follow(leftLeadMotor, Constants.LEFT_MOTOR_INVERT);
    rightFollower.follow(rightLeadMotor, Constants.RIGHT_MOTOR_INVERT);
    
    // Set all Motors to Brake Mode
    leftLeadMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightLeadMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Set the default command for a subsystem here.
    m_drive = new DifferentialDrive(leftLeadMotor, rightLeadMotor);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(DoubleSupplier forward, DoubleSupplier turn){
    // Originally these parameters were flipped, 
    // But it appeared to be a naming issue
    // This is the order that worked... if not please change

    m_drive.arcadeDrive(forward.getAsDouble(), turn.getAsDouble());
  }

  public void run(double forward, double turn){
    // Originally these parameters were flipped, 
    // But it appeared to be a naming issue
    // This is the order that worked... if not please change
    
    m_drive.arcadeDrive(forward, turn);
  }

}
