package frc.robot.subsystems.limelight;

public enum StreamMode {
    BOTH(0),
    LIMELIGHT_CAMERA(1),
    SECONDARY_CAMERA(2);

    private int stream;

    private StreamMode(int stream) {
        this.stream = stream;
    }

    public int getStream() {
        return stream;
    }
}
