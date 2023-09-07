package frc.robot.subsystems.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightBase {
    public String name = "limelight";
    public double cameraAngle;
    public double cameraHeight;
    public double targetWidth;

    public double frontDistance;

    public NetworkTable limelightTable;

    public LimelightBase(String name, double cameraAngle, double cameraHeight, double frontDistance) {
        this.name = name;

        this.cameraAngle = cameraAngle;
        this.cameraHeight = cameraHeight;

        this.frontDistance = frontDistance;

        limelightTable = NetworkTableInstance.getDefault().getTable(name);
    }


    /**
     * Gets the value of the given key from the limelight
     * @param key 
     * @return value of the key
     */
    public double getValue(LimelightKey key) {
        return limelightTable.getEntry(key.getKey()).getDouble(0.0);
    }

    
    /**
     * Gets the average value of the given key from the limelight
     * @param key
     * @param numSamples
     * @return average value of the key
     */
    public double getValueAverage(LimelightKey key, int numSamples){
        double runningSum = 0;

        for(int i = 0; i < numSamples; i++){
            runningSum += getValue(key);
        }
        
        return runningSum / numSamples;
    }

    /**
     * Checks if the limelight has a valid target
     * @return true if the limelight has a valid target
     */
    public boolean hasValidTarget(){
        return getValue(LimelightKey.VALID_TARGET) > 0.99;
    }

    /**
     * Calculates the distance to the top target using the limelight
     * @param targetHeight
     * @return distance to the top of the target
     */
    public double calculateDistToTopTarget(double targetHeight){
        if (!hasValidTarget())
            return -1;

        double ty = getValue(LimelightKey.VERTICAL_OFFSET) * Math.PI / 180;
        double tx = getValue(LimelightKey.HORIZONTAL_OFFSET) * Math.PI / 180;
        return (targetHeight - cameraHeight) / (Math.tan(ty + cameraAngle) * Math.cos(tx)) - frontDistance;
    }

    /**
     * Calculates the distance to the ground target using the limelight
     * @param targetHeight
     * @return distance to the bottom of the target
     */
    public double calculateDistToGroundTarget(double targetHeight){
        if (!hasValidTarget())
            return -1;

        double ty = getValue(LimelightKey.VERTICAL_OFFSET) * Math.PI / 180;
        return (-targetHeight + cameraHeight) * Math.tan(ty + cameraAngle) - frontDistance;
    }

    /**
     * Sets the LED mode of the limelight
     * @param mode
     */
    public void setLEDMode(LEDMode mode){
        limelightTable.getEntry("ledMode").setNumber(mode.LEDValue());
    }

    /**
     * Sets the camera mode of the limelight
     * @param mode
     */
    public void setStreamMode(StreamMode mode){
        limelightTable.getEntry("stream").setNumber(mode.getStream());
    }

    /**
     * Sets the pipeline of the limelight
     * @param pipeline
     */
    public void setPipeline(Pipeline pipeline){
        limelightTable.getEntry("pipeline").setNumber(pipeline.getPipeline());
    }

    /**
     * @return the current pipeline of the limelight
     */
    public double getSelectedPipeline(){
        return limelightTable.getEntry("pipeline").getDouble(0);
    };
}
