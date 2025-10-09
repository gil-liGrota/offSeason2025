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

public class AutonomousRoutinesRed {

        public static Command driveToPoseInCorrectAlliance(Swerve drive, Pose2d pose, boolean proccessorSide) {
                // if (proccessorSide) {
                // pose = new Pose2d(pose.getX(), FieldConstants.fieldWidth - pose.getY(),
                // pose.getRotation().unaryMinus());
                // }
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
                                putL4NoProccessorSide(drive, elevator, arm, transfer, 2, true),

                                // go to coral station
                                goToNoProccessorCoralStation(drive, elevator, arm, transfer),

                                // 5 L4 right
                                Commands.parallel(
                                                driveToPoseInCorrectAlliance(drive,
                                                                new Pose2d(14.55, 1.73, Rotation2d.fromDegrees(124)),
                                                                false).withTimeout(2),
                                                riffCommands.L4()),
                                putL4NoProccessorSide(drive, elevator, arm, transfer, 1, true));
        }

        public static Command putL4NoProccessorSide(Swerve drive, Elevator elevator, CoralArm arm,
                        Transfer transfer, int branch, boolean isRightBranch) {
                Pose2d targetPose;
                // Pose2d attackPose = FieldConstants.Reef.redAttackPoses[branch];

                // SmartDashboard.putString("attackPose " + branch, attackPose.toString());
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
                                                // SwerveCommands.joystickDriveRobotRelative(drive, () -> 0, () -> 0,
                                                // () -> 0).withTimeout(0.5),
                                                Commands.sequence(
                                                                CoralArmCommands.goToPosition(arm, 1.2),
                                                                Commands.waitSeconds(0.2),
                                                                TransferCommands.riffOutake(transfer, arm)
                                                                                .withTimeout(0.5),
                                                                CoralArmCommands.goToPosition(arm,
                                                                                OPEN_ARM_POSITION))));
        }

        public static Command putL4ProccessorSide(Swerve drive, Elevator elevator, CoralArm arm,
                        Transfer transfer, int branch, boolean isRightBranch) {
                Pose2d targetPose;
                // Pose2d attackPose = FieldConstants.Reef.redAttackPoses[branch];

                // SmartDashboard.putString("attackPose " + branch, attackPose.toString());
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
                                                // SwerveCommands.joystickDriveRobotRelative(drive, () -> 0, () -> 0,
                                                // () -> 0).withTimeout(0.5),
                                                Commands.sequence(
                                                                CoralArmCommands.goToPosition(arm, 1.2),
                                                                Commands.waitSeconds(0.2),
                                                                TransferCommands.riffOutake(transfer, arm)
                                                                                .withTimeout(0.5),
                                                                CoralArmCommands.goToPosition(arm,
                                                                                OPEN_ARM_POSITION))));
        }

        public static Command putL4Middel(Swerve drive, Elevator elevator, CoralArm arm,
                        Transfer transfer, int branch, boolean isRightBranch) {
                Pose2d targetPose;
                // Pose2d attackPose = FieldConstants.Reef.redAttackPoses[branch];

                // SmartDashboard.putString("attackPose " + branch, attackPose.toString());
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
                                driveToPoseInCorrectAlliance(drive, targetPose, false).withTimeout(2.5),
                                Commands.parallel(
                                                SwerveCommands.joystickDriveRobotRelative(drive, () -> 0, () -> 0,
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

        public static Command goToProccessorCoralStation(Swerve drive, Elevator elevator, CoralArm arm,
                        Transfer transfer) {
                RiffCommands riffCommands = new RiffCommands(elevator, arm, transfer);
                return (Commands.parallel(
                                Commands.sequence(
                                                Commands.waitSeconds(0.5),
                                                riffCommands.coralIntakePos()),
                                driveToPoseInCorrectAlliance(drive,
                                                redProccessorCoralStation,
                                                false))
                                .until(transfer.getIO()::isCoralIn));
        }

        public static Command goToProccessor(Swerve drive, Elevator elevator, CoralArm arm,
                        Transfer transfer) {
                RiffCommands riffCommands = new RiffCommands(elevator, arm, transfer);
                return (Commands.parallel(
                                Commands.sequence(
                                                Commands.waitSeconds(0.5),
                                                riffCommands.coralIntakePos()),
                                driveToPoseInCorrectAlliance(drive,
                                                redProccessorCoralStation,
                                                false))
                                .until(transfer.getIO()::isCoralIn));
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        public static Command putL4ProccessorSideRED(Swerve drive, Elevator elevator, CoralArm arm, Transfer transfer) {

                boolean proccessorSide = false;
                RiffCommands riffCommands = new RiffCommands(elevator, arm, transfer);
                return Commands.sequence(
                                // 6 L4 right
                                Commands.parallel(
                                                driveToPoseInCorrectAlliance(drive,
                                                                new Pose2d(11.67, 6.12, Rotation2d.fromDegrees(-45.79)),
                                                                true)
                                                                .withTimeout(2),
                                                riffCommands.L4()),
                                putL4ProccessorSide(drive, elevator, arm, transfer, 4, false),

                                // go to coral station
                                goToProccessorCoralStation(drive, elevator, arm, transfer),

                                // 5 L4 right
                                Commands.parallel(
                                                driveToPoseInCorrectAlliance(drive,
                                                                new Pose2d(14.69, 6.12,
                                                                                Rotation2d.fromDegrees(-131.13)),
                                                                true).withTimeout(2),
                                                riffCommands.L4()),
                                putL4ProccessorSide(drive, elevator, arm, transfer, 5, false));

        }

        public static Command putL4MiddelRED(Swerve drive, Elevator elevator, CoralArm arm, Transfer transfer) {
                RiffCommands riffCommands = new RiffCommands(elevator, arm, transfer);

                return Commands.sequence(
                                riffCommands.L4(),
                                putL4Middel(drive, elevator, arm, transfer, 3, true),
                                SwerveCommands.driveBackSlow(drive).withTimeout(1)

                );

        }
}
