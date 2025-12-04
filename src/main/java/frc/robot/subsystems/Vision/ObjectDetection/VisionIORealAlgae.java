package frc.robot.subsystems.Vision.ObjectDetection;

import frc.robot.subsystems.Vision.VisionConstants;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

public class VisionIORealAlgae {
    PhotonCamera camera;

    public VisionIORealAlgae(String name) {
        camera = new PhotonCamera(name);
    }

//    public void updateInputs(VisionIOInputs inputs) {
//        inputs.pipelineType = PipelineType.OBJECT_DETECTION_WITH_DISTANCE_ESTIMATION;
//        inputs.connected = camera.isConnected();
//
//
//    }

    // calculated by dist = (focal length * real width) / perceived width
    private double getDistanceToAlgae(double targetWidthInPixels) {
        if(camera.getCameraMatrix().isPresent())
            return (camera.getCameraMatrix().get().get(0,0)
                    * VisionConstants.ObjectDetectionConstants.ALGAE_DIAMETER)
                    / targetWidthInPixels;
        Logger.recordOutput("vision/errors", "Camera matrix not present");
        return 0;
    }
}
