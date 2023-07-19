// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
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
  private RelativeEncoder leftEncoderFollower;
  private RelativeEncoder rightEncoder;
  private RelativeEncoder rightEncoderFollower;

  // Create Differential Drive
  private final DifferentialDrive drive;

  // Create PID Controllers
  private final SparkMaxPIDController _drivingPIDController;
  private final SparkMaxPIDController _turningPIDController;

  // Create Odometry
  private final Odometry odometry;

  // Create Filters for Slew Rate Limiting
  SlewRateLimiter turnFilter = new SlewRateLimiter(DrivetrainConstants.SLEW_RATE_TURN);
  SlewRateLimiter driveFilter = new SlewRateLimiter(DrivetrainConstants.SLEW_RATE_DRIVE);

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
    leftEncoderFollower = leftFollower.getEncoder();
    rightEncoder = rightLeadMotor.getEncoder();
    rightEncoderFollower = rightFollower.getEncoder();

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
    drive = new DifferentialDrive(leftLeadMotor, rightLeadMotor);

    // Flash is burnt in robotContainer... incinerateMotors()

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.execute();
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

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public RelativeEncoder getLeftEncoder() {
    return leftEncoder;
  }

  public RelativeEncoder getRightEncoder() {
    return rightEncoder;
  }

  public void run(double forward, double turn) {      
    drive.arcadeDrive(driveFilter.calculate(-forward), 
                        turnFilter.calculate(turn));
  }
}