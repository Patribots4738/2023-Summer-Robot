// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.Odometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  // Create Motors
  private final CANSparkMax leftLeadMotor;
  private final CANSparkMax rightLeadMotor;
  private final CANSparkMax leftFollower;
  private final CANSparkMax rightFollower;

  // Create Encoders
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  // Create Differential Drive
  private final DifferentialDrive drive;

  // Create Odometry
  private final Odometry odometry;

  // Create Filters for Slew Rate Limiting
  SlewRateLimiter turnFilter = new SlewRateLimiter(DrivetrainConstants.SLEW_RATE_TURN_POSITIVE, DrivetrainConstants.SLEW_RATE_TURN_NEGATIVE, 0);
  SlewRateLimiter driveFilter = new SlewRateLimiter(DrivetrainConstants.SLEW_RATE_DRIVE_POSITIVE, DrivetrainConstants.SLEW_RATE_DRIVE_NEGATIVE, 0);

  private double forward;
  private double turn;

  // Create Gyro
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  public Drivetrain() {

    // Initialize motors
    leftLeadMotor = new CANSparkMax(DrivetrainConstants.LEFT_MOTOR_FRONT_CAN_ID,
        CANSparkMaxLowLevel.MotorType.kBrushless);
    rightLeadMotor = new CANSparkMax(DrivetrainConstants.RIGHT_MOTOR_FRONT_CAN_ID,
        CANSparkMaxLowLevel.MotorType.kBrushless);
    leftFollower = new CANSparkMax(DrivetrainConstants.LEFT_MOTOR_FOLLOWER_CAN_ID,
        CANSparkMaxLowLevel.MotorType.kBrushless);
    rightFollower = new CANSparkMax(DrivetrainConstants.RIGHT_MOTOR_FOLLOWER_CAN_ID,
        CANSparkMaxLowLevel.MotorType.kBrushless);

    Constants.SPARK_LIST.add(leftLeadMotor);
    Constants.SPARK_LIST.add(rightLeadMotor);
    Constants.SPARK_LIST.add(leftFollower);
    Constants.SPARK_LIST.add(rightFollower);

    leftEncoder = leftLeadMotor.getEncoder();
    rightEncoder = rightLeadMotor.getEncoder();

    odometry = new Odometry(this);

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

    // follow the lead motors
    leftFollower.follow(leftLeadMotor, DrivetrainConstants.LEFT_MOTOR_INVERT);
    rightFollower.follow(rightLeadMotor, DrivetrainConstants.RIGHT_MOTOR_INVERT);

    // Set the default command for a subsystem here.
    drive = new DifferentialDrive(leftLeadMotor, rightLeadMotor);

    // Flash is burnt in robotContainer... incinerateMotors()
    super.register();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.execute();

    drive.arcadeDrive(
        driveFilter.calculate(forward), 
        turn
    );
  }

  public Odometry getOdometry() {
    return odometry;
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getAngle());
  }

  public double getAngle() {
    return gyro.getAngle();
  }

  public RelativeEncoder getLeftEncoder() {
    return leftEncoder;
  }

  public RelativeEncoder getRightEncoder() {
    return rightEncoder;
  }

  public void modeBreak(){
    for (CANSparkMax spark : Constants.SPARK_LIST) {
      spark.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
  }

  public void run(double forward, double turn) {
    this.forward = -forward;
    this.turn = turn;
  }

  public void modeCoast() {
    for (CANSparkMax spark : Constants.SPARK_LIST) {
      spark.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
  }
}