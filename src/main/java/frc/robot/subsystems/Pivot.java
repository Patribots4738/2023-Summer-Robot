package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    pivotFollower = new CANSparkMax(PivotConstants.PIVOT_FOLLOWER_CAN_ID, MotorType.kBrushless);

    // Restore the motors to factory defaults
    pivotLead.restoreFactoryDefaults();
    pivotFollower.restoreFactoryDefaults();

    pivotFollower.follow(pivotLead, true);

    pivotPIDController = pivotLead.getPIDController();

    pivotEncoder = pivotLead.getAbsoluteEncoder(Type.kDutyCycle);

    // Convert the encoder position from rotations to degrees
    pivotEncoder.setPositionConversionFactor(PivotConstants.PIVOT_POSITION_ENCODER_FACTOR);
    pivotPIDController.setPositionPIDWrappingEnabled(true);
    
    pivotLead.setSmartCurrentLimit(PivotConstants.PIVOT_SMART_CURRENT_LIMIT);

  }

  // TODO: add this to the periodic method of the robot container
  public void periodic() {
    pivotPIDController.setReference(this.desiredRotation, ControlType.kPosition);
  }

  /**
   * Get the current rotation of the pivot
   * 
   * @return the current rotation of the pivot
   */
  public double getRotationDegrees() {
    return desiredRotation;
  }

  public double getEncoderPosition() {
    return pivotEncoder.getPosition();
  }

  /**
   * Set the rotation of the pivot, in degrees
   * 
   * @param placementPositionDegrees the rotation of the pivot
   */
  public void setDesiredRotation(double placementPositionDegrees) {
    if (placementPositionDegrees > PivotConstants.PIVOT_HIGH_LIMIT_DEGREES) {
      placementPositionDegrees = PivotConstants.PIVOT_HIGH_LIMIT_DEGREES;
    } else if (placementPositionDegrees < PivotConstants.PIVOT_LOW_LIMIT_DEGREES) {
      placementPositionDegrees = PivotConstants.PIVOT_LOW_LIMIT_DEGREES;
    }
    desiredRotation = placementPositionDegrees;
  
    periodic();
}

}
