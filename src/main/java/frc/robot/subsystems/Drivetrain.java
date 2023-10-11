// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
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

    private final DifferentialDriveKinematics kinematics;

    // Create Odometry
    private final Odometry odometry;

    // Create Filters for Slew Rate Limiting
    SlewRateLimiter turnFilter;
    SlewRateLimiter driveFilter;

    // Create Ramsete Controller
    private final RamseteController ramseteController;

    private double forward;
    private double turn;

    private SimpleMotorFeedforward feedforward;

    private PIDController leftPIDController;
    private PIDController rightPIDController;

    private final ADIS16470_IMU gyro;
    private Field2d field;

    private static Drivetrain instance;

    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;

    public Drivetrain() {

        kinematics = DrivetrainConstants.DRIVE_KINEMATICS;
        feedforward = new SimpleMotorFeedforward(DrivetrainConstants.kS, DrivetrainConstants.kV, DrivetrainConstants.kA);
        leftPIDController = new PIDController(DrivetrainConstants.DRIVING_P, DrivetrainConstants.DRIVING_I, DrivetrainConstants.DRIVING_D);
        rightPIDController = new PIDController(DrivetrainConstants.DRIVING_P, DrivetrainConstants.DRIVING_I, DrivetrainConstants.DRIVING_D);
        
        turnFilter = new SlewRateLimiter(DrivetrainConstants.SLEW_RATE_TURN_POSITIVE,
                DrivetrainConstants.SLEW_RATE_TURN_NEGATIVE, 0);
        driveFilter = new SlewRateLimiter(DrivetrainConstants.SLEW_RATE_DRIVE_POSITIVE,
                DrivetrainConstants.SLEW_RATE_DRIVE_NEGATIVE, 0);
        ramseteController = new RamseteController(ControllerConstants.BETA,
                ControllerConstants.ZETA);

        gyro = new ADIS16470_IMU();
        field = new Field2d();

        // Initialize motors
        leftLeadMotor = new CANSparkMax(DrivetrainConstants.LEFT_MOTOR_FRONT_CAN_ID,
                CANSparkMaxLowLevel.MotorType.kBrushless);
        rightLeadMotor = new CANSparkMax(DrivetrainConstants.RIGHT_MOTOR_FRONT_CAN_ID,
                CANSparkMaxLowLevel.MotorType.kBrushless);
        leftFollower = new CANSparkMax(DrivetrainConstants.LEFT_MOTOR_FOLLOWER_CAN_ID,
                CANSparkMaxLowLevel.MotorType.kBrushless);
        rightFollower = new CANSparkMax(DrivetrainConstants.RIGHT_MOTOR_FOLLOWER_CAN_ID,
                CANSparkMaxLowLevel.MotorType.kBrushless);

        leftLeadMotor.restoreFactoryDefaults();
        rightLeadMotor.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

        leftEncoder = leftLeadMotor.getEncoder();
        leftEncoder.setPositionConversionFactor(DrivetrainConstants.DRIVING_ENCODER_POS_FACTOR);
        leftEncoder.setPositionConversionFactor(DrivetrainConstants.DRIVING_ENCODER_VEL_FACTOR);
        rightEncoder = rightLeadMotor.getEncoder();
        rightEncoder.setPositionConversionFactor(DrivetrainConstants.DRIVING_ENCODER_POS_FACTOR);
        rightEncoder.setVelocityConversionFactor(DrivetrainConstants.DRIVING_ENCODER_VEL_FACTOR);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        odometry = new Odometry(
                getYaw(),
                leftEncoder.getPosition(),
                rightEncoder.getPosition(),
                new Pose2d());

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

        Constants.SPARK_LIST.add(leftLeadMotor);
        Constants.SPARK_LIST.add(rightLeadMotor);
        Constants.SPARK_LIST.add(leftFollower);
        Constants.SPARK_LIST.add(rightFollower);

        leftMotors = new MotorControllerGroup(leftFollower, leftLeadMotor);
        rightMotors = new MotorControllerGroup(rightFollower, rightLeadMotor);

        // Set the default command for a subsystem here.
        drive = new DifferentialDrive(leftMotors, rightMotors);

        // create a field for displaying robot position on the dashboard
        SmartDashboard.putData("Field", this.field);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // if (rightEncoder.getPosition() != 0) System.out.println(rightEncoder.getPosition());

        drive.arcadeDrive(
          driveFilter.calculate(forward),
          turn);
          
        // drive.tankDrive(
        //   forward,
        //   turn);
          
        odometry.update(
                Rotation2d.fromDegrees(getGyroAngle()),
                leftEncoder.getPosition(),
                rightEncoder.getPosition());
        
        field.setRobotPose(odometry.getPoseMeters());
    }

    public Odometry getOdometry() {
        return odometry;
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    
    public double getGyroAngle() {
        return gyro.getAngle();
    }
    
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(getGyroAngle());
    }

    public Rotation2d getPitch() {

        Rotation2d pitchRotation2d = Rotation2d
            .fromDegrees(gyro.getXComplementaryAngle() - ((gyro.getXComplementaryAngle() > 0) ? 180 : -180));
    
        return pitchRotation2d;
    
      }

      public Rotation2d getRoll() {

        Rotation2d rollRotation2d = Rotation2d
            .fromDegrees(gyro.getYComplementaryAngle() - ((gyro.getYComplementaryAngle() > 0) ? 180 : -180));
    
        return rollRotation2d;
    
      }
    
    public RelativeEncoder getLeftEncoder() {
        return leftEncoder;
    }

    public RelativeEncoder getRightEncoder() {
        return rightEncoder;
    }

    public void modeBreak() {
        for (CANSparkMax spark : Constants.SPARK_LIST) {
            spark.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }
    }
    
    public void drive(double forward, double turn) {
        this.forward = -forward;
        this.turn = turn;
        drive.feed();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
      // leftMotors.setVoltage(leftVolts);
      // rightMotors.setVoltage(rightVolts);

      leftMotors.set(leftVolts);
      rightMotors.set(rightVolts);

      drive.feed();
    }

    public void modeCoast() {
        for (CANSparkMax spark : Constants.SPARK_LIST) {
            spark.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        double leftOutput = speeds.leftMetersPerSecond;
        double rightOutput = speeds.rightMetersPerSecond;

        leftLeadMotor.set(leftOutput);
        rightLeadMotor.set(rightOutput);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public RamseteController getRamseteController() {
        return ramseteController;
    }

    public PIDController getLeftPIDController() {
        return leftPIDController;
    }

    public PIDController getRightPIDController() {
        return rightPIDController;
    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    public void setOutputSpeeds(double leftOutput, double rightOutput) {
        this.setSpeeds(
                new DifferentialDriveWheelSpeeds(leftOutput, rightOutput));
    }

    public static synchronized Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

    public void stop() {
        drive.stopMotor();
    }


    
}