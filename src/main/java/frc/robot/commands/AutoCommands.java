package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.CoralArm.CoralArm;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.drive.FieldConstants;
import frc.robot.subsystems.drive.Swerve;

public class AutoCommands {

        RiffCommands reefCommands;
        Swerve drive;
        CoralArm arm;
        Elevator lift;
        Transfer transfer;

        public AutoCommands(Swerve drive, Elevator elevator, CoralArm arm, Transfer transfer) {
                reefCommands = new RiffCommands(elevator, arm, transfer);
                this.drive = drive;
                this.arm = arm;
                this.lift = elevator;
                this.transfer = transfer;
        }

        public static Command driveToPoseInCorrectAlliance(Swerve drive, Pose2d pose) {
                return new ConditionalCommand(new SwerveCommands.DriveToPosition(drive, pose),
                                new SwerveCommands.DriveToPosition(drive,
                                                new Pose2d(FieldConstants.fieldLength - pose.getX(),
                                                                FieldConstants.fieldWidth - pose.getY(),
                                                                Rotation2d.fromDegrees(180
                                                                                + pose.getRotation().getDegrees()))),
                                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
                                .finallyDo((a) -> {
                                        drive.stop();
                                });
        }

        public Command leftSideAuto() {
                // Pose2d target = FieldConstants.Reef.blueRightBranches[3];
                return Commands.sequence(
                                Commands.parallel(
                                                reefCommands.L4(),
                                                driveToPoseInCorrectAlliance(drive,
                                                                new Pose2d(new Translation2d(6.45, 6.3),
                                                                                new Rotation2d(-140))))
                                                .until(() -> SwerveCommands.LocateToReefCommand.isAnyReefCloseEnough(
                                                                drive.getPose(),
                                                                false)),
                                placeCoralInReef(4, false),
                                Commands.parallel(
                                                driveToPoseInCorrectAlliance(drive,
                                                                new Pose2d(new Translation2d(4.1, 5.1),
                                                                                new Rotation2d(Units.degreesToRadians(
                                                                                                -35))))),
                                intakeFromLeftFeeder(),
                                Commands.parallel(
                                                driveToPoseInCorrectAlliance(drive,
                                                                new Pose2d(new Translation2d(2, 6),
                                                                                new Rotation2d(-44))))
                                                .until(() -> SwerveCommands.LocateToReefCommand.isAnyReefCloseEnough(
                                                                drive.getPose(),
                                                                false)),
                                placeCoralInReef(2, false),
                                intakeFromLeftFeeder());
        }

        public Command intakeFromLeftFeeder() {
                return Commands.parallel(reefCommands.coralIntakePos(),
                                driveToPoseInCorrectAlliance(drive, new Pose2d(new Translation2d(1.2, 7.2),
                                                new Rotation2d(Units.degreesToRadians(-60)))));
        }

        public Command placeCoralInReef(int reefLevel, boolean leftSide) {
                return Commands.sequence(
                                Commands.parallel(new SwerveCommands.LocateToReefCommand(drive, null, leftSide),
                                                reefCommands.Lx(reefLevel)),
                                TransferCommands.riffOutake(transfer, arm));
        }
}