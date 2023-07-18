package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.PivotConstants;

public class Pivot extends PIDSubsystem {
    private double desiredRotation;

    CANSparkMax pivotLead;
    CANSparkMax pivotFollower;

    private final AbsoluteEncoder pivotEncoder;

    public Pivot() {
        super(new PIDController(PivotConstants.PIVOT_P, PivotConstants.PIVOT_I, PivotConstants.PIVOT_D));
        this.desiredRotation = 0.0;

        // Initialize the motors
        pivotLead = new CANSparkMax(PivotConstants.PIVOT_LEAD_CAN_ID, MotorType.kBrushless);
        pivotFollower = new CANSparkMax(PivotConstants.PIVOT_FOLLOWER_CAN_ID, MotorType.kBrushless);

        // Restore the motors to factory defaults
        pivotLead.restoreFactoryDefaults();
        pivotFollower.restoreFactoryDefaults();

        pivotLead.setInverted(false);

        pivotFollower.follow(pivotLead, true);
        pivotEncoder = pivotLead.getAbsoluteEncoder(Type.kDutyCycle);

        // Convert the encoder position from rotations to degrees
        pivotEncoder.setPositionConversionFactor(PivotConstants.PIVOT_POSITION_ENCODER_FACTOR);

        pivotLead.setSmartCurrentLimit(PivotConstants.PIVOT_SMART_CURRENT_LIMIT);

        // The constant found here can be found in
        // REV Hardware client when the arm is pointed straight up
        // Then, depending on if the value is more or less than 180,
        // Add or subtract 180 to the value
        pivotEncoder.setZeroOffset(PivotConstants.ORIGINAL_ZERO_OFFSET + PivotConstants.ZERO_OFFSET_OFFSET);

        pivotLead.burnFlash();
        pivotFollower.burnFlash();

    }

    @Override
    protected void useOutput(double output, double setpoint) {
        pivotLead.set(output);
    }

    /**
     * Get the current rotation of the pivot
     * 
     * @return the current rotation of the pivot
     */
    public double getDesiredPositionDegrees() {
        return desiredRotation;
    }

    public double getEncoderPositionDegrees() {
        return pivotEncoder.getPosition();
    }

    @Override
    protected double getMeasurement() { // returns degrees
        return getEncoderPositionDegrees();
    }

    /**
     * Set the rotation of the pivot, in degrees
     * 
     * @param placementPositionDegrees the rotation of the pivot
     */
    public void setDesiredRotation(double placementPositionDegrees) {
        MathUtil.clamp(
            placementPositionDegrees, 
            PivotConstants.PIVOT_LOW_LIMIT_DEGREES,
            PivotConstants.PIVOT_HIGH_LIMIT_DEGREES
        );

        enable();
        super.setSetpoint(this.desiredRotation);
    }
}
