package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase{
  private double speed;

  // TODO: Declare Motors and Encoders

  public Claw() {
    // TODO: make this the value of the motor at start
    speed = 0.0;

    // TODO: Initialize Motors and Encoders
  }

  public void setSpeed(double speed) {
    // TODO: Set the speed of the motors
    this.speed = speed;
  }

}
