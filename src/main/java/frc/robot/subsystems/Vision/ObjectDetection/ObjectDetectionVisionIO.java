package frc.robot.subsystems.Vision.ObjectDetection;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionVisionIO {
    @AutoLog
    public static class ObjectDetectionVisionIOInputs {
        public boolean connected = false;
        public Detection[] detections = new Detection[0];
    }

    public void updateInputs(ObjectDetectionVisionIOInputs inputs);
}
