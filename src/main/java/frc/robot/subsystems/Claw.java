package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
  private double speed;

  private final CANSparkMax leadMotor;
  private final CANSparkMax followerMotor;


  public Claw() {
    speed = 0.0;

    leadMotor = new CANSparkMax(ClawConstants.CLAW_LEAD_CAN_ID, CANSparkMax.MotorType.kBrushless);
    Constants.SPARK_LIST.add(leadMotor);
    followerMotor = new CANSparkMax(ClawConstants.CLAW_FOLLOWER_CAN_ID, CANSparkMax.MotorType.kBrushless);
    Constants.SPARK_LIST.add(followerMotor);

    leadMotor.restoreFactoryDefaults();
    followerMotor.restoreFactoryDefaults();

    leadMotor.setSmartCurrentLimit(ClawConstants.CLAW_STALL_CURRENT_LIMIT, ClawConstants.CLAW_FREE_CURRENT_LIMIT);
    
    followerMotor.follow(leadMotor, true);

    // Flash is burnt in robotContainer... incinerateMotors()
  }

  // Set the speed of the motors, positive is out, negative is in
  // range: -1, 1
  public void setSpeed(double speed) {
    this.speed = speed;
    periodic();
  }

  // TODO: add this to the periodic method of the robot container
  public void periodic() {
    leadMotor.set(speed);
  }
}
