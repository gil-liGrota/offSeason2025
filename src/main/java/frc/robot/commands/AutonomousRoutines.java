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
                                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);
        }

        public static Command putReef(int level, Swerve drive, Elevator elevator, CoralArm arm, Transfer transfer,
                        boolean proccessorSide) {

                Pose2d[] pose1 = new Pose2d[] { new Pose2d(13.5, 1.5, new Rotation2d(Math.PI)),
                                new Pose2d(16.4, 1.1, Rotation2d.fromDegrees(125)) };

                RiffCommands riffCommands = new RiffCommands(elevator, arm, transfer);
                return Commands.sequence(
                                ElevatorCommands.goToPosition(elevator, 16.0).withTimeout(1),
                                Commands.parallel(
                                                driveToPoseInCorrectAlliance(drive,
                                                                FieldConstants.Reef.redRightBranches[2],
                                                                proccessorSide).withTimeout(5),
                                                new RiffCommands(elevator, arm, transfer).Lx(level).withTimeout(2)),
                                Commands.race(
                                                Commands.sequence(
                                                                new RiffCommands(elevator, arm, transfer).Lx(level)
                                                                                .unless(() -> level < 3)
                                                                                .withTimeout(2),
                                                                TransferCommands.riffOutake(transfer, arm)
                                                                                .withTimeout(0.5)),
                                                SwerveCommands.joystickDriveRobotRelative(drive, () -> 0.4, () -> 0,
                                                                () -> 0)),
                                CoralArmCommands.goToPosition(arm, OPEN_ARM_POSITION),
                                Commands.parallel(
                                                ElevatorCommands.closeElevator(elevator).withTimeout(1),
                                                SwerveCommands.driveBackSlow(drive).withTimeout(1)),
                                Commands.parallel(
                                                riffCommands.coralIntakePos(),
                                                /*
                                                 * driveToPoseInCorrectAlliance(drive,
                                                 * new Pose2d(16.4, 1.1, Rotation2d.fromDegrees(125)),
                                                 * proccessorSide)
                                                 */
                                                SwerveCommands.joystickDriveRobotRelative(drive, () -> 0, () -> 0,
                                                                () -> 0.5).withTimeout(1))

                );
        }

}
