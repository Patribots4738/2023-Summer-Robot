package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

public class Pivot {
  private double rotation;

  // TODO: Declare Motors and Encoders
  // HINT: Copy the formatting of the drivetrain
  CANSparkMax pivotLead;
  CANSparkMax pivotFollower;


  public Pivot() {
    // TODO: make this the value of the motor at start
    this.rotation = 0.0; 
    
    // TODO: Initialize Motors and Encoders
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
