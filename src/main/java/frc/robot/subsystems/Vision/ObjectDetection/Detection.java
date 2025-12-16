package frc.robot.subsystems.Vision.ObjectDetection;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Vision.VisionConstants;

import java.util.Optional;
import frc.robot.subsystems.Vision.VisionConstants.TargetType;


public record Detection(
        TargetType targetType,
        double targetWidth,
        double targetHeight,
        double cameraRelativeTargetRotation,
        Optional<Translation2d> cameraRelativeTargetTranslation
){}
