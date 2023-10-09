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
import edu.wpi.first.wpilibj.RobotController;
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

    private final DifferentialDriveKinematics kinematics = DrivetrainConstants.DRIVE_KINEMATICS;

    // Create Odometry
    private final Odometry odometry;

    // Create Filters for Slew Rate Limiting
    SlewRateLimiter turnFilter = new SlewRateLimiter(DrivetrainConstants.SLEW_RATE_TURN_POSITIVE,
            DrivetrainConstants.SLEW_RATE_TURN_NEGATIVE, 0);
    SlewRateLimiter driveFilter = new SlewRateLimiter(DrivetrainConstants.SLEW_RATE_DRIVE_POSITIVE,
            DrivetrainConstants.SLEW_RATE_DRIVE_NEGATIVE, 0);

    // Create Ramsete Controller
    private final RamseteController ramseteController = new RamseteController(ControllerConstants.BETA,
            ControllerConstants.ZETA);

    private double forward;
    private double turn;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DrivetrainConstants.kS, DrivetrainConstants.kV, DrivetrainConstants.kA);

    private PIDController leftPIDController = new PIDController(DrivetrainConstants.DRIVING_P, DrivetrainConstants.DRIVING_I, DrivetrainConstants.DRIVING_D);
    private PIDController rightPIDController = new PIDController(DrivetrainConstants.DRIVING_P, DrivetrainConstants.DRIVING_I, DrivetrainConstants.DRIVING_D);

    private final ADIS16470_IMU gyro = new ADIS16470_IMU();
    private Field2d field = new Field2d();

    private static Drivetrain instance;

    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;

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

        leftMotors = new MotorControllerGroup(leftFollower, leftLeadMotor);
        rightMotors = new MotorControllerGroup(rightFollower, rightLeadMotor);

        leftEncoder = leftLeadMotor.getEncoder();
        rightEncoder = rightLeadMotor.getEncoder();

        odometry = new Odometry(
                Rotation2d.fromDegrees(getAngle()),
                leftEncoder.getPosition(),
                rightEncoder.getPosition(),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

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
        drive = new DifferentialDrive(leftMotors, rightMotors);


        // create a field for displaying robot position on the dashboard
        SmartDashboard.putData("Field", this.field);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
        drive.arcadeDrive(
          driveFilter.calculate(forward),
          turn);
          
        // drive.tankDrive(
        //   forward,
        //   turn);
          
          odometry.update(
                  Rotation2d.fromDegrees(getAngle()),
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

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getAngle() {
        return gyro.getAngle();
    }

    public void zeroGyro() {
        gyro.reset();
    }

    // TODO: How do we use this? ADIS16740 doesnt have a  setYaw(double angle) method
    public void zeroGyro(double angle) {}

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
      leftMotors.setVoltage(leftVolts);
      rightMotors.setVoltage(rightVolts);
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