package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
  private double rotation;

  // TODO: Declare Motors and Encoders
  // HINT: Copy the formatting of the drivetrain
  CANSparkMax pivotLead;
  CANSparkMax pivotFollower;


  public Pivot() {
    // TODO: make this the value of the motor at start
    this.rotation = 0.0; 
    
    // TODO: Initialize Motors and Encoders
    pivotLead = new CANSparkMax(0, MotorType.kBrushless);
    pivotFollower = new CANSparkMax(1, MotorType.kBrushless);

    pivotFollower.follow(pivotLead);
  }


  
  /**
   * Get the current rotation of the pivot
   * @return the current rotation of the pivot
   */
  public double getRotationDegrees() {
    return this.rotation;
  }

  /**
   * Set the rotation of the pivot, in degrees
   * @param placementPositionDegrees the rotation of the pivot
   */
  public void setRotation(double placementPositionDegrees) {
    this.rotation = placementPositionDegrees;
  }
  
}
