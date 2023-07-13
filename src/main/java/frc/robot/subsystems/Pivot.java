package frc.robot.subsystems;

public class Pivot {
  private double rotation;

  // TODO: Declare Motors and Encoders

  public Pivot() {
    // TODO: make this the value of the motor at start
    this.rotation = 0; 
    
    // TODO: Initialize Motors and Encoders
  }

  public double getRotation() {
    return this.rotation;
  }

  public void setRotation(double placementPosition) {
    this.rotation = placementPosition;
  }
  
}
