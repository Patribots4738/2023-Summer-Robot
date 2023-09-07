package frc.robot.subsystems.limelight;

public enum LEDMode {
    ON(3), OFF(1), BLINK(2);

    private int value;

    private LEDMode(int value) {
        this.value = value;
    }

    public int LEDValue() {
        return value;
    }
}
