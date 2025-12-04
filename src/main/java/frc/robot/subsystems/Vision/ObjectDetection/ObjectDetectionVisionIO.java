package frc.robot.subsystems.Vision.ObjectDetection;

public interface ObjectDetectionVisionIO {
    public static class ObjectDetectionVisionIOInputs {
        public boolean connected = false;
        public double targetXAngle = 0.0;
        public double targetYAngle = 0.0;
        public double targetDistance = 0.0;
        public boolean targetDetected = false;
    }

    public void updateInputs(ObjectDetectionVisionIOInputs inputs);
}
