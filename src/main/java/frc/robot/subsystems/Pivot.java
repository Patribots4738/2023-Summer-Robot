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
import frc.robot.Constants.PlacementConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Pivot extends SubsystemBase implements Loggable {
    private double desiredRotation;

    CANSparkMax pivotLead;
    CANSparkMax pivotFollower;

    @Config
    SparkMaxPIDController pivotPIDController;

    private final AbsoluteEncoder pivotEncoder;

    @Log.Graph(visibleTime = 20)
    private double encoderPositionDegrees;

    @Config
    public static double PIVOT_P = 0.02;

    @Config
    public static double PIVOT_I = 0;

    @Config
    public static double PIVOT_D = 0.02;

    public static int placementIndex;

    public Pivot() {
        this.desiredRotation = 0.0;
        Pivot.placementIndex = PlacementConstants.RESET_INDEX;

        // Initialize the motors
        pivotLead = new CANSparkMax(PivotConstants.PIVOT_LEAD_CAN_ID, MotorType.kBrushless);
        pivotFollower = new CANSparkMax(PivotConstants.PIVOT_FOLLOWER_CAN_ID, MotorType.kBrushless);

        // Restore the motors to factory defaults
        pivotLead.restoreFactoryDefaults();
        pivotFollower.restoreFactoryDefaults();

        pivotFollower.follow(pivotLead, true);

        pivotPIDController = pivotLead.getPIDController();

        pivotEncoder = pivotLead.getAbsoluteEncoder(Type.kDutyCycle);

        pivotPIDController.setP(PIVOT_P);
        pivotPIDController.setI(PIVOT_I);
        pivotPIDController.setD(PIVOT_D);

        // Convert the encoder position from rotations to degrees
        pivotEncoder.setPositionConversionFactor(PivotConstants.PIVOT_POSITION_ENCODER_FACTOR);

        pivotPIDController.setFeedbackDevice(pivotEncoder);

        pivotLead.setSmartCurrentLimit(PivotConstants.PIVOT_SMART_CURRENT_LIMIT);
        pivotPIDController.setOutputRange(-0.3, 0.3);
        // Flash is burnt in robotContainer... incinerateMotors()
        Constants.SPARK_LIST.add(pivotLead);
        Constants.SPARK_LIST.add(pivotFollower);

        setDesiredRotation(PlacementConstants.PLACEMENT_POSITIONS_FRONT[PlacementConstants.RESET_INDEX]);
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
        encoderPositionDegrees = pivotEncoder.getPosition();
        return encoderPositionDegrees;
    }

    /**
     * Set the rotation of the pivot, in degrees
     * 
     * @param placementPositionDegrees the rotation of the pivot
     */
    public void setDesiredRotation(double placementPositionDegrees) {
        // Clamp the desired rotation to the limits of the pivot
        this.desiredRotation = MathUtil.clamp(placementPositionDegrees, PivotConstants.PIVOT_LOW_LIMIT_DEGREES,
                PivotConstants.PIVOT_HIGH_LIMIT_DEGREES);
        // System.out.println("desired Rotation:" + this.desiredRotation);
        // pivotPIDController.setReference(this.desiredRotation, ControlType.kPosition);
    }

    public void setArmHigh(boolean isBackwards) {
        Pivot.placementIndex = PlacementConstants.HIGH_INDEX;
        setDesiredRotation((isBackwards) ? PlacementConstants.PLACEMENT_POSITIONS_BACK[PlacementConstants.HIGH_INDEX]
                : PlacementConstants.PLACEMENT_POSITIONS_FRONT[PlacementConstants.HIGH_INDEX]);
    }

    public void setArmLow(boolean isBackwards) {
        Pivot.placementIndex = PlacementConstants.LOW_INDEX;
        setDesiredRotation((isBackwards) ? PlacementConstants.PLACEMENT_POSITIONS_BACK[PlacementConstants.LOW_INDEX]
                : PlacementConstants.PLACEMENT_POSITIONS_FRONT[PlacementConstants.LOW_INDEX]);
    }

    public void setArmMid(boolean isBackwards) {
        Pivot.placementIndex = PlacementConstants.MID_INDEX;
        setDesiredRotation((isBackwards) ? PlacementConstants.PLACEMENT_POSITIONS_BACK[PlacementConstants.MID_INDEX]
                : PlacementConstants.PLACEMENT_POSITIONS_FRONT[PlacementConstants.MID_INDEX]);
    }

    public void setArmReset() {
        Pivot.placementIndex = PlacementConstants.RESET_INDEX;
        setDesiredRotation(PlacementConstants.RESET_PLACEMENT);
    }

}