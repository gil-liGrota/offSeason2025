package frc.robot.subsystems.Vision.ObjectDetection;

import frc.robot.subsystems.Vision.VisionConstants;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class VisionIORealAlgae implements ObjectDetectionVisionIO {
    final PhotonCamera camera;
    final String name;

    public VisionIORealAlgae(String name) {
        this.name = name;
        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(ObjectDetectionVisionIOInputs inputs) {
        List<PhotonPipelineResult> allUnreadResults = camera.getAllUnreadResults();
        PhotonPipelineResult latestResult = allUnreadResults.get(allUnreadResults.size() - 1);

        Detection[] detections = new Detection[latestResult.getTargets().size()];
        for(int i = 0; i < detections.length; i++) {
            PhotonTrackedTarget target = latestResult.getTargets().get(i);

            if(target.getDetectedObjectClassID() == VisionConstants.TargetType.ALGAE.getClassId()) {

            }
            // TODO: Implement this properly
        }

        inputs.pipelineName = name;
        inputs.connected = camera.isConnected();
        inputs.detections = detections;
    }

    private boolean isSameObject(PhotonTrackedTarget targetA, PhotonTrackedTarget targetB) {
        // TODO: implement better object matching logic
        return true;
    }

    // calculated by dist = (focal length * real width) / perceived width
    private double getDistanceToAlgaeMeters(double targetWidthInPixels) {
        if(camera.getCameraMatrix().isPresent())
            return (camera.getCameraMatrix().get().get(0,0)
                    * VisionConstants.ObjectDetectionConstants.ALGAE_DIAMETER_METERS)
                    / targetWidthInPixels;
        Logger.recordOutput("vision/errors", "Camera matrix not present");
        return 0;
    }
}
