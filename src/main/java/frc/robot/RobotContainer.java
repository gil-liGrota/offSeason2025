// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.CoralArm.CoralArmConstants.CLOSE_ARM_POSITION;
import static frc.robot.subsystems.CoralArm.CoralArmConstants.L1_ARM_POSITION;
import static frc.robot.subsystems.CoralArm.CoralArmConstants.L2_ARM_POSITION;
import static frc.robot.subsystems.CoralArm.CoralArmConstants.L4_ARM_POSITION;
import static frc.robot.subsystems.Elevator.ElevatorConstants.L1_ELEVATOR_POSITION;
import static frc.robot.subsystems.Elevator.ElevatorConstants.L2_ELEVATOR_POSITION;
import static frc.robot.subsystems.Elevator.ElevatorConstants.L4_ELEVATOR_POSITION;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.POM_lib.Joysticks.PomXboxController;
import frc.robot.commands.CoralArmCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.commands.TransferCommands;
import frc.robot.subsystems.CoralArm.CoralArm;
import frc.robot.subsystems.CoralArm.CoralArmIOReal;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorReal;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.Transfer.TransferIOReal;
import frc.robot.subsystems.drive.GyroIOPigeon;
import frc.robot.subsystems.drive.ModuleIOReal;
import frc.robot.subsystems.drive.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Subsystems
        Elevator elevator;
        CoralArm coralArm;
        Transfer transfer;
        Swerve drive;
        // Controller
        private final PomXboxController driverController = new PomXboxController(0);
        private final PomXboxController operatorController = new PomXboxController(1);
        private final PomXboxController manualController = new PomXboxController(2);

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        private boolean isRelative;

        private SwerveDriveSimulation driveSimulation = null;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                elevator = new Elevator(new ElevatorReal(() -> false));
                                coralArm = new CoralArm(new CoralArmIOReal());
                                transfer = new Transfer(new TransferIOReal());
                                drive = new Swerve(new GyroIOPigeon(),
                                                new ModuleIOReal(0),
                                                new ModuleIOReal(1),
                                                new ModuleIOReal(2),
                                                new ModuleIOReal(3));
                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                break;
                }

                SendableChooser<Command> c = new SendableChooser<>();

                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", c); // TODO use auto builder

                autoChooser.addDefaultOption("none", null);

                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                // driver:
                isRelative = true;
                drive.setDefaultCommand(
                                SwerveCommands.joystickDrive(
                                                drive,
                                                () -> driverController.getLeftY(),
                                                () -> driverController.getLeftX(),
                                                () -> driverController.getRightX()));

                driverController.y().onTrue(drive.resetGyroCommand());

                // operator:
                // close arm and elevator
                operatorController.RB().onTrue(ElevatorCommands.closeElevator(elevator));
                operatorController.LB().onTrue(CoralArmCommands.goToPosition(coralArm, CLOSE_ARM_POSITION));

                // coral intake position
                operatorController.b().onTrue(ElevatorCommands.goToPosition(elevator, 2.0)
                                .alongWith(CoralArmCommands.setVoltage(coralArm, -0.2)));

                // L4
                operatorController.y().onTrue(CoralArmCommands.goToPosition(coralArm,
                                L4_ARM_POSITION)
                                .until(() -> coralArm.getIO().getPosition() >= L4_ARM_POSITION)
                                .andThen(ElevatorCommands.goToPosition(elevator, L4_ELEVATOR_POSITION)));

                // L3
                operatorController.x().onTrue(CoralArmCommands.goToPosition(coralArm, 0.95));

                // L2
                operatorController.a().onTrue(ElevatorCommands.goToPosition(elevator,
                                L4_ELEVATOR_POSITION)
                                .until(() -> elevator.getIO().getPosition() - 0.4 >= L2_ELEVATOR_POSITION)
                                .andThen(CoralArmCommands.goToPosition(coralArm, L2_ARM_POSITION)));

                // // L1
                // operatorController.LB().onTrue(ElevatorCommands.goToPosition(elevator,
                // L2_ELEVATOR_POSITION)
                // .until(() -> elevator.getIO().getPosition() - 0.4 >= L1_ELEVATOR_POSITION)
                // .andThen(CoralArmCommands.goToPosition(coralArm, L1_ARM_POSITION)));

                // intake, l2, l1
                operatorController.PovUp().whileTrue(TransferCommands.coralintake(transfer,
                                5));
                operatorController.PovLeft().whileTrue(TransferCommands.coralintake(transfer,
                                12));

                // outake, l4, l3
                operatorController.PovDown().whileTrue(TransferCommands.coralintake(transfer,
                                -5));
                operatorController.PovRight().whileTrue(TransferCommands.coralintake(transfer,
                                -12));

                // manuale:
                // manuale elevator
                manualController.rightTrigger().whileTrue(ElevatorCommands.openElevatorManual(elevator,
                                4));
                manualController.leftTrigger().whileTrue(ElevatorCommands.closeElevatorManual(elevator,
                                -1));
                // elevator
                manualController.a().onTrue(ElevatorCommands.closeElevator(elevator));
                manualController.y().onTrue(ElevatorCommands.goToPosition(elevator,
                                L4_ELEVATOR_POSITION));

                // manuale coral arm
                manualController.PovLeft().whileTrue(CoralArmCommands.setVoltage(coralArm,
                                1));
                manualController.PovRight().whileTrue(CoralArmCommands.setVoltage(coralArm,
                                -1));
                // coral arm
                manualController.b().onTrue(CoralArmCommands.goToPosition(coralArm,
                                L4_ARM_POSITION));
                manualController.x().onTrue(CoralArmCommands.goToPosition(coralArm,
                                CLOSE_ARM_POSITION));

        }

        public void displaSimFieldToAdvantageScope() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;

                Logger.recordOutput(
                                "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
                Logger.recordOutput(
                                "FieldSimulation/Notes", SimulatedArena.getInstance().getGamePiecesArrayByType("Note"));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }
}
