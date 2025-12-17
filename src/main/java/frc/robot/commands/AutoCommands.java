package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.drive.FieldConstants;
import frc.robot.subsystems.drive.Swerve;

public class AutoCommands {
    public static Command driveToPoseInCorrectAlliance(Swerve drive, Pose2d pose) {
        return new ConditionalCommand(new SwerveCommands.DriveToPosition(drive, pose),
                new SwerveCommands.DriveToPosition(drive,
                        new Pose2d(FieldConstants.fieldLength - pose.getX(),
                                FieldConstants.fieldWidth - pose.getY(),
                                Rotation2d.fromDegrees(180
                                        + pose.getRotation().getDegrees()))),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
                .finallyDo((a) -> {
                    drive.stop();
                });
    }

}
