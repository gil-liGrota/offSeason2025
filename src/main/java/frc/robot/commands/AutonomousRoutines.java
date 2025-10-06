package frc.robot.commands;

import static frc.robot.subsystems.CoralArm.CoralArmConstants.L4_ARM_POSITION;
import static frc.robot.subsystems.CoralArm.CoralArmConstants.OPEN_ARM_POSITION;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.CoralArm.CoralArm;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.drive.FieldConstants;
import static frc.robot.subsystems.drive.FieldConstants.CoralStation.*;
import frc.robot.subsystems.drive.Swerve;

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
                                                driveToPoseInCorrectAlliance(drive,
                                                                new Pose2d(12.36, 2.00, Rotation2d.fromDegrees(59.12)),
                                                                false)
                                                                .withTimeout(2),
                                                riffCommands.L4()),
                                putL4(drive, elevator, arm, transfer, 2, true),

                                // go to coral station
                                goToNoProccessorCoralStation(drive, elevator, arm, transfer),

                                // 5 L4 right
                                Commands.parallel(
                                                driveToPoseInCorrectAlliance(drive,
                                                                new Pose2d(14.55, 1.73, Rotation2d.fromDegrees(124)),
                                                                false).withTimeout(2),
                                                riffCommands.L4()),
                                putL4(drive, elevator, arm, transfer, 1, true));
                // // go to coral station
                // goToNoProccessorCoralStation(drive, elevator, arm, transfer),
                // // // 5 L4 left
                // driveToPoseInCorrectAlliance(drive,
                // new Pose2d(14.44, 2.01, Rotation2d.fromDegrees(130)),
                // false).withTimeout(2),
                // putL4(drive, elevator, arm, transfer, 1, false));
        }

        public static Command putL4(Swerve drive, Elevator elevator, CoralArm arm,
                        Transfer transfer, int branch, boolean isRightBranch) {
                Pose2d targetPose;
                Pose2d attackPose = FieldConstants.Reef.redAttackPoses[branch];

                SmartDashboard.putString("attackPose " + branch, attackPose.toString());
                RiffCommands riffCommands = new RiffCommands(elevator, arm, transfer);

                if (isRightBranch) {
                        targetPose = FieldConstants.Reef.redRightBranches[branch];
                } else {
                        targetPose = FieldConstants.Reef.redLeftBranches[branch];
                }

                return Commands.sequence(
                                // Commands.parallel(
                                // // driveToPoseInCorrectAlliance(drive, attackPose,
                                // // false).withTimeout(2),
                                // riffCommands.L4()),
                                driveToPoseInCorrectAlliance(drive, targetPose, false).withTimeout(1.7),
                                Commands.parallel(
                                                SwerveCommands.joystickDriveRobotRelative(drive, () -> 0.4, () -> 0.1,
                                                                () -> 0).withTimeout(0.5),
                                                Commands.sequence(
                                                                CoralArmCommands.goToPosition(arm, 1.2),
                                                                Commands.waitSeconds(0.2),
                                                                TransferCommands.riffOutake(transfer, arm)
                                                                                .withTimeout(0.5),
                                                                CoralArmCommands.goToPosition(arm,
                                                                                OPEN_ARM_POSITION))));
        }

        public static Command goToNoProccessorCoralStation(Swerve drive, Elevator elevator, CoralArm arm,
                        Transfer transfer) {
                RiffCommands riffCommands = new RiffCommands(elevator, arm, transfer);
                return (Commands.parallel(
                                Commands.sequence(
                                                Commands.waitSeconds(0.5),
                                                riffCommands.coralIntakePos()),
                                driveToPoseInCorrectAlliance(drive,
                                                redNoProccessorCoralStation,
                                                false))
                                .until(transfer.getIO()::isCoralIn));
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        public static Command putL4ProccessorSideRED(Swerve drive, Elevator elevator, CoralArm arm, Transfer transfer) {

                Pose2d[] pose1 = new Pose2d[] { new Pose2d(13.5, 1.5, new Rotation2d(Math.PI)),
                                new Pose2d(16.4, 1.1, Rotation2d.fromDegrees(125)) };

                boolean proccessorSide = true;
                RiffCommands riffCommands = new RiffCommands(elevator, arm, transfer);
                return Commands.sequence(
                                // 2 L4 right
                                Commands.parallel(
                                                driveToPoseInCorrectAlliance(drive,
                                                                new Pose2d(14.44, 2.01, Rotation2d.fromDegrees(130)),
                                                                false).withTimeout(2),
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
