package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import static frc.robot.subsystems.CoralArm.CoralArmConstants.*;

public class AutonomousRoutines {

        public static Command driveToPoseInCorrectAlliance(Swerve drive, Pose2d pose, boolean proccessorSide) {
                if (proccessorSide) {
                        pose = new Pose2d(pose.getX(), FieldConstants.fieldWidth - pose.getY(),
                                        pose.getRotation().unaryMinus());
                }
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

        public static Command putL4NotProccessorSideRED(Swerve drive, Elevator elevator, CoralArm arm,
                        Transfer transfer) {

                Pose2d[] pose1 = new Pose2d[] { new Pose2d(13.5, 1.5, new Rotation2d(Math.PI)),
                                new Pose2d(16.4, 1.1, Rotation2d.fromDegrees(125)) };

                boolean proccessorSide = false;
                RiffCommands riffCommands = new RiffCommands(elevator, arm, transfer);
                return Commands.sequence(
                                // 6 L4 right
                                Commands.parallel(
                                                SwerveCommands.driveForwardSlowRight(drive).withTimeout(0.5),
                                                riffCommands.L4().withTimeout(2)),
                                driveToPoseInCorrectAlliance(drive,
                                                FieldConstants.Reef.redRightBranches[2], proccessorSide).withTimeout(5),
                                Commands.race(
                                                Commands.sequence(
                                                                CoralArmCommands.goToPosition(arm, L4_ARM_POSITION),
                                                                TransferCommands.riffOutake(transfer, arm)
                                                                                .withTimeout(0.5)),
                                                SwerveCommands.joystickDriveRobotRelative(drive, () -> 0.4, () -> 0,
                                                                () -> 0)),
                                // go to coral station
                                Commands.parallel(
                                                CoralArmCommands.goToPosition(arm, OPEN_ARM_POSITION),
                                                ElevatorCommands.closeElevator(elevator).withTimeout(1),
                                                SwerveCommands.driveBackSlow(drive).withTimeout(2)),
                                Commands.parallel(
                                                riffCommands.coralIntakePos(),
                                                driveToPoseInCorrectAlliance(drive,
                                                                new Pose2d(16.6, 0.85, Rotation2d.fromDegrees(125)),
                                                                false).until(transfer.getIO()::isCoralIn))
                                                .until(() -> elevator.getIO().getPosition() > 13),
                                // 5 L4 right
                                Commands.parallel(
                                                SwerveCommands.driveForwardSlowRight(drive).withTimeout(1),
                                                riffCommands.L4()),
                                driveToPoseInCorrectAlliance(drive,
                                                FieldConstants.Reef.redRightBranches[1],
                                                proccessorSide).withTimeout(5),
                                CoralArmCommands.goToPosition(arm, L4_ARM_POSITION),
                                TransferCommands.autoIntakeCoral(transfer).withTimeout(0.5),
                                CoralArmCommands.goToPosition(arm, OPEN_ARM_POSITION),
                                // go to coral station
                                Commands.parallel(
                                                ElevatorCommands.closeElevator(elevator).withTimeout(1),
                                                SwerveCommands.driveBackSlow(drive).withTimeout(1)),
                                Commands.parallel(
                                                riffCommands.coralIntakePos(),
                                                driveToPoseInCorrectAlliance(drive,
                                                                new Pose2d(16.6, 0.85, Rotation2d.fromDegrees(125)),
                                                                false).until(transfer.getIO()::isCoralIn))
                                                .until(() -> elevator.getIO().getPosition() > 13),
                                // // 5 L4 left
                                Commands.parallel(
                                                SwerveCommands.driveForwardSlowLeft(drive).withTimeout(1),
                                                riffCommands.L4()),
                                driveToPoseInCorrectAlliance(drive,
                                                FieldConstants.Reef.redLeftBranches[1],
                                                proccessorSide).withTimeout(3),
                                SwerveCommands.joystickDriveRobotRelative(drive, () -> 0.3, () -> 0,
                                                () -> 0).withTimeout(1),
                                CoralArmCommands.goToPosition(arm, L4_ARM_POSITION),
                                TransferCommands.autoIntakeCoral(transfer).withTimeout(0.5),
                                CoralArmCommands.goToPosition(arm, OPEN_ARM_POSITION)

                );
        }

        public static Command putL4ProccessorSideRED(Swerve drive, Elevator elevator, CoralArm arm, Transfer transfer) {

                Pose2d[] pose1 = new Pose2d[] { new Pose2d(13.5, 1.5, new Rotation2d(Math.PI)),
                                new Pose2d(16.4, 1.1, Rotation2d.fromDegrees(125)) };

                boolean proccessorSide = true;
                RiffCommands riffCommands = new RiffCommands(elevator, arm, transfer);
                return Commands.sequence(
                                // 2 L4 right
                                Commands.parallel(
                                                SwerveCommands.driveForwardSlowRight(drive).withTimeout(0.5),
                                                riffCommands.L4().withTimeout(2)),
                                driveToPoseInCorrectAlliance(drive,
                                                FieldConstants.Reef.redRightBranches[4], proccessorSide).withTimeout(5),
                                Commands.race(
                                                Commands.sequence(
                                                                CoralArmCommands.goToPosition(arm, L4_ARM_POSITION),
                                                                TransferCommands.riffOutake(transfer, arm)
                                                                                .withTimeout(0.5)),
                                                SwerveCommands.joystickDriveRobotRelative(drive, () -> 0.4, () -> 0,
                                                                () -> 0)),
                                // go to coral station
                                Commands.parallel(
                                                CoralArmCommands.goToPosition(arm, OPEN_ARM_POSITION),
                                                ElevatorCommands.closeElevator(elevator).withTimeout(1),
                                                SwerveCommands.driveBackSlow(drive).withTimeout(2)),
                                Commands.parallel(
                                                riffCommands.coralIntakePos(),
                                                driveToPoseInCorrectAlliance(drive,
                                                                new Pose2d(16.6, 0.85, Rotation2d.fromDegrees(125)),
                                                                false).until(transfer.getIO()::isCoralIn))
                                                .until(() -> elevator.getIO().getPosition() > 13),
                                // 3 L4 right
                                Commands.parallel(
                                                SwerveCommands.driveForwardSlowRight(drive).withTimeout(1),
                                                riffCommands.L4()),
                                driveToPoseInCorrectAlliance(drive,
                                                FieldConstants.Reef.redRightBranches[5],
                                                proccessorSide).withTimeout(5),
                                CoralArmCommands.goToPosition(arm, L4_ARM_POSITION),
                                TransferCommands.autoIntakeCoral(transfer).withTimeout(0.5),
                                CoralArmCommands.goToPosition(arm, OPEN_ARM_POSITION),
                                // go to coral station
                                Commands.parallel(
                                                ElevatorCommands.closeElevator(elevator).withTimeout(1),
                                                SwerveCommands.driveBackSlow(drive).withTimeout(1)),
                                Commands.parallel(
                                                riffCommands.coralIntakePos(),
                                                driveToPoseInCorrectAlliance(drive,
                                                                new Pose2d(16.6, 0.85, Rotation2d.fromDegrees(125)),
                                                                false).until(transfer.getIO()::isCoralIn))
                                                .until(() -> elevator.getIO().getPosition() > 13),
                                // // 3 L4 left
                                Commands.parallel(
                                                SwerveCommands.driveForwardSlowLeft(drive).withTimeout(1),
                                                riffCommands.L4()),
                                driveToPoseInCorrectAlliance(drive,
                                                FieldConstants.Reef.redLeftBranches[5],
                                                proccessorSide).withTimeout(3),
                                SwerveCommands.joystickDriveRobotRelative(drive, () -> 0.3, () -> 0,
                                                () -> 0).withTimeout(1),
                                CoralArmCommands.goToPosition(arm, L4_ARM_POSITION),
                                TransferCommands.autoIntakeCoral(transfer).withTimeout(0.5),
                                CoralArmCommands.goToPosition(arm, OPEN_ARM_POSITION)

                );
        }

        public static Command putL4MiddelRED(Swerve drive, Elevator elevator, CoralArm arm, Transfer transfer) {
                Pose2d[] pose1 = new Pose2d[] { new Pose2d(13.5, 1.5, new Rotation2d(Math.PI)),
                                new Pose2d(16.4, 1.1, Rotation2d.fromDegrees(125)) };

                boolean proccessorSide = true;
                RiffCommands riffCommands = new RiffCommands(elevator, arm, transfer);

                return Commands.sequence(
                                // 1 L4 right
                                Commands.parallel(
                                                SwerveCommands.driveForwardSlowRight(drive).withTimeout(0.5),
                                                riffCommands.L4().withTimeout(2)),
                                driveToPoseInCorrectAlliance(drive,
                                                FieldConstants.Reef.redRightBranches[3], proccessorSide).withTimeout(5),
                                Commands.race(
                                                Commands.sequence(
                                                                CoralArmCommands.goToPosition(arm, L4_ARM_POSITION),
                                                                TransferCommands.riffOutake(transfer, arm)
                                                                                .withTimeout(0.5)),
                                                SwerveCommands.joystickDriveRobotRelative(drive, () -> 0.4, () -> 0,
                                                                () -> 0)),
                                Commands.parallel(
                                                ElevatorCommands.closeElevator(elevator).withTimeout(1),
                                                SwerveCommands.driveBackSlow(drive).withTimeout(1)),
                                // 1 alage outake
                                Commands.parallel(
                                                riffCommands.AlgaeIntakeHigh().withTimeout(2)),
                                driveToPoseInCorrectAlliance(drive,
                                                FieldConstants.Reef.redRightBranches[3], proccessorSide).withTimeout(5)// FIXME
                                                                                                                       // move
                                                                                                                       // to
                                                                                                                       // the
                                                                                                                       // middel

                );
        }
}
