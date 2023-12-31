package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.PlacementConstants;

public class Claw extends SubsystemBase {

    private double speed;

    private final CANSparkMax leadMotor;
    private final CANSparkMax followerMotor;

    private final SlewRateLimiter filter;

    private static Claw instance;

    public static Claw getInstance() {
        if (instance == null) {
            instance = new Claw();
        }
        return instance;
    }

    public Claw() {
        speed = 0.0;

        filter = new SlewRateLimiter(ClawConstants.SLEW_POSITIVE_LIMIT, ClawConstants.SLEW_NEGATIVE_LIMIT, 0);

        leadMotor = new CANSparkMax(ClawConstants.CLAW_LEAD_CAN_ID, CANSparkMax.MotorType.kBrushless);
        Constants.SPARK_LIST.add(leadMotor);
        followerMotor = new CANSparkMax(ClawConstants.CLAW_FOLLOWER_CAN_ID, CANSparkMax.MotorType.kBrushless);
        Constants.SPARK_LIST.add(followerMotor);

        leadMotor.restoreFactoryDefaults();
        followerMotor.restoreFactoryDefaults();

        leadMotor.setSmartCurrentLimit(ClawConstants.CLAW_STALL_CURRENT_LIMIT, ClawConstants.CLAW_FREE_CURRENT_LIMIT);
        followerMotor.setSmartCurrentLimit(ClawConstants.CLAW_STALL_CURRENT_LIMIT, ClawConstants.CLAW_FREE_CURRENT_LIMIT);

        // followerMotor.follow(leadMotor, true);

        // Flash is burnt in robotContainer... incinerateMotors()

    }

    // Set the speed of the motors, positive is out, negative is in
    // range: -1, 1
    public void setSpeed(double speed) {
        leadMotor.setSmartCurrentLimit(
          speed < 0 ? ClawConstants.CLAW_OUTTAKE_CURRENT_LIMIT : ClawConstants.CLAW_STALL_CURRENT_LIMIT,
          ClawConstants.CLAW_FREE_CURRENT_LIMIT);

        followerMotor.setSmartCurrentLimit(
          speed < 0 ? ClawConstants.CLAW_OUTTAKE_CURRENT_LIMIT : ClawConstants.CLAW_STALL_CURRENT_LIMIT,
          ClawConstants.CLAW_FREE_CURRENT_LIMIT);

        // if we are intaking, only set the speed if it is faster than the current speed
        this.speed = (speed > 0 && this.speed > speed) ? this.speed : speed;
    }

    public void setSpeed(int index, boolean isBackwards){
        this.speed = (!isBackwards) ? PlacementConstants.PLACEMENT_SPEEDS_FRONT[index] : PlacementConstants.PLACEMENT_SPEEDS_BACK[index];
        setSpeed(speed);
    }

    public void periodic() {
        double calculatedSpeed = filter.calculate(speed);
        leadMotor.set(-calculatedSpeed);
        followerMotor.set(calculatedSpeed);
    }
}

