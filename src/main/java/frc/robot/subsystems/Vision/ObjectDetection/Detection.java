package frc.robot.subsystems.Vision.ObjectDetection;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.Optional;


public class Detection {
    public static enum TargetType {
        ALGAE,
        CORAL
    }

    private TargetType targetType;
    private Optional<Translation2d> cameraRelativeTargetTranslation;
    private double cameraRelativeTargetRotation;
    private double targetWidth, targetHeight;

    public Detection(TargetType targetType, double targetWidth, double targetHeight, double cameraRelativeTargetRotation) {
        this.targetType = targetType;
        this.cameraRelativeTargetRotation = cameraRelativeTargetRotation;
        this.targetWidth = targetWidth;
        this.targetHeight = targetHeight;
        cameraRelativeTargetTranslation = Optional.empty();
    }

    public Detection(TargetType targetType, double targetWidth, double targetHeight, double cameraRelativeTargetRotation, Translation2d translationFromCamera) {
        this(targetType, targetWidth, targetHeight, cameraRelativeTargetRotation);
        this.cameraRelativeTargetTranslation = Optional.ofNullable(translationFromCamera);
    }

    public TargetType getTargetType() {
        return targetType;
    }

    public void setTargetType(TargetType targetType) {
        this.targetType = targetType;
    }

    public Optional<Translation2d> getCameraRelativeTargetTranslation() {
        return cameraRelativeTargetTranslation;
    }

    public void setCameraRelativeTargetTranslation(Optional<Translation2d> cameraRelativeTargetTranslation) {
        this.cameraRelativeTargetTranslation = cameraRelativeTargetTranslation;
    }

    public double getCameraRelativeTargetRotation() {
        return cameraRelativeTargetRotation;
    }

    public void setCameraRelativeTargetRotation(double cameraRelativeTargetRotation) {
        this.cameraRelativeTargetRotation = cameraRelativeTargetRotation;
    }

    public double getTargetWidth() {
        return targetWidth;
    }

    public void setTargetWidth(double targetWidth) {
        this.targetWidth = targetWidth;
    }

    public double getTargetHeight() {
        return targetHeight;
    }

    public void setTargetHeight(double targetHeight) {
        this.targetHeight = targetHeight;
    }

    @Override
    public String toString() {
        return "Detection{" +
                "targetType=" + targetType +
                ", cameraRelativeTargetTranslation=" + cameraRelativeTargetTranslation +
                ", cameraRelativeTargetRotation=" + cameraRelativeTargetRotation +
                ", targetWidth=" + targetWidth +
                ", targetHeight=" + targetHeight +
                '}';
    }
}
