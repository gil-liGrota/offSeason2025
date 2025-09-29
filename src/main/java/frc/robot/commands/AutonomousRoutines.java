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

                return Commands.sequence(
                                driveToPoseInCorrectAlliance(drive, FieldConstants.Reef.redRightBranches[2], // Drive to
                                                                                                             // reef
                                                proccessorSide).withTimeout(4) // 4 second timeout
                                                .alongWith(new RiffCommands(elevator, arm, transfer).Lx(level)), // Along
                                                                                                                 // with
                                                                                                                 // elevator
                                                                                                                 // and
                                                                                                                 // arm
                                TransferCommands.riffOutake(transfer, arm).withTimeout(0.5) // Outake coral

                );
        }

}
