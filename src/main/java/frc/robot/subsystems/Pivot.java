package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  private double desiredRotation;

  CANSparkMax pivotLead;
  CANSparkMax pivotFollower;

  SparkMaxPIDController pivotPIDController;

  private final AbsoluteEncoder pivotEncoder;

  public Pivot() {
    this.desiredRotation = 0.0;

    // Initialize the motors
    pivotLead = new CANSparkMax(PivotConstants.PIVOT_LEAD_CAN_ID, MotorType.kBrushless);
    Constants.SPARK_LIST.add(pivotLead);
    pivotFollower = new CANSparkMax(PivotConstants.PIVOT_FOLLOWER_CAN_ID, MotorType.kBrushless);
    Constants.SPARK_LIST.add(pivotFollower);

    // Restore the motors to factory defaults
    pivotLead.restoreFactoryDefaults();
    pivotFollower.restoreFactoryDefaults();

    pivotFollower.follow(pivotLead, true);

    pivotPIDController = pivotLead.getPIDController();

    pivotEncoder = pivotLead.getAbsoluteEncoder(Type.kDutyCycle);

    // Convert the encoder position from rotations to degrees
    pivotEncoder.setPositionConversionFactor(PivotConstants.PIVOT_POSITION_ENCODER_FACTOR);

    // TODO: Comment me out once I run once just in case something happens
    // We want this false, but false is default... but it might be true currently
    // just run once and delete :)
    pivotPIDController.setPositionPIDWrappingEnabled(false);
    pivotPIDController.setFeedbackDevice(pivotEncoder);

    pivotLead.setSmartCurrentLimit(PivotConstants.PIVOT_SMART_CURRENT_LIMIT);

    // Flash is burnt in robotContainer... incinerateMotors()
  }

  /**
   * Get the current rotation of the pivot
   * 
   * @return the current rotation of the pivot
   */
  public double getRotationDegrees() {
    return desiredRotation;
  }

  public double getEncoderPositionDegrees() {
    return pivotEncoder.getPosition();
  }

  /**
   * Set the rotation of the pivot, in degrees
   * 
   * @param placementPositionDegrees the rotation of the pivot
   */
  public void setDesiredRotation(double placementPositionDegrees) {
    // Clamp the desired rotation to the limits of the pivot
    MathUtil.clamp(placementPositionDegrees, PivotConstants.PIVOT_LOW_LIMIT_DEGREES, PivotConstants.PIVOT_LOW_LIMIT_DEGREES);

    pivotPIDController.setReference(this.desiredRotation, ControlType.kPosition);
  }

}