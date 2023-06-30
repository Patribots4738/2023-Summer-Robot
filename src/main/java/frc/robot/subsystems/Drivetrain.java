// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final CANSparkMax leftLeadMotor;
  private final CANSparkMax rightLeadMotor;
  private final CANSparkMax leftFollower;
  private final CANSparkMax rightFollower;

  private final DifferentialDrive m_drive;
  private final SparkMaxPIDController _drivingPIDController;
  private final SparkMaxPIDController _turningPIDController;

  public Drivetrain() {

    // initialize motors
    leftLeadMotor = new CANSparkMax(DrivetrainConstants.LEFT_MOTOR_FRONT_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightLeadMotor = new CANSparkMax(DrivetrainConstants.RIGHT_MOTOR_FRONT_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftFollower = new CANSparkMax(DrivetrainConstants.LEFT_MOTOR_FOLLOWER_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightFollower = new CANSparkMax(DrivetrainConstants.RIGHT_MOTOR_FOLLOWER_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    
    leftLeadMotor.restoreFactoryDefaults();
    rightLeadMotor.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    leftLeadMotor.setInverted(true);
    rightLeadMotor.setInverted(false);

    // Set all Motors to Brake Mode
    leftLeadMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightLeadMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Set PID Controller
    _drivingPIDController = leftLeadMotor.getPIDController();
    _turningPIDController = rightLeadMotor.getPIDController();

    _drivingPIDController.setP(DrivetrainConstants.DRIVING_P);
    _drivingPIDController.setI(DrivetrainConstants.DRIVING_I);
    _drivingPIDController.setD(DrivetrainConstants.DRIVING_D);

    _turningPIDController.setP(DrivetrainConstants.TURNING_P);
    _turningPIDController.setI(DrivetrainConstants.TURNING_I);
    _turningPIDController.setD(DrivetrainConstants.TURNING_D);
    
    // follow the lead motors
    leftFollower.follow(leftLeadMotor, DrivetrainConstants.LEFT_MOTOR_INVERT);
    rightFollower.follow(rightLeadMotor, DrivetrainConstants.RIGHT_MOTOR_INVERT);

    // Set the default command for a subsystem here.
    m_drive = new DifferentialDrive(leftLeadMotor, rightLeadMotor);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double forward, double turn){
    m_drive.arcadeDrive(forward, turn);
  }
}
