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

import static frc.robot.subsystems.CoralArm.CoralArmConstants.BRAKE_SWITCH;
import static frc.robot.subsystems.CoralArm.CoralArmConstants.CLOSE_ARM_POSITION;
import static frc.robot.subsystems.CoralArm.CoralArmConstants.L2_ARM_POSITION;
import static frc.robot.subsystems.CoralArm.CoralArmConstants.L3_ARM_POSITION;
import static frc.robot.subsystems.CoralArm.CoralArmConstants.L4_ARM_POSITION;
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
import frc.robot.POM_lib.sensors.POMDigitalInput;
import frc.robot.commands.CoralArmCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.RiffCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.commands.TransferCommands;
import frc.robot.subsystems.CoralArm.CoralArm;
import frc.robot.subsystems.CoralArm.CoralArmIOReal;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorReal;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.Transfer.TransferIOReal;
import frc.robot.subsystems.Vision.VisionIOReal;
import frc.robot.subsystems.Vision.VisionSubsystem;
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
        VisionSubsystem vision;

        // Controller
        private final PomXboxController driverController = new PomXboxController(0);
        private final PomXboxController operatorController = new PomXboxController(1);
        private final PomXboxController manualController = new PomXboxController(2);

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        private boolean isRelative;
        private POMDigitalInput brakeSwitch = new POMDigitalInput(BRAKE_SWITCH);
        RiffCommands riffCommands;

        private SwerveDriveSimulation driveSimulation = null;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                elevator = new Elevator(new ElevatorReal(() -> false, brakeSwitch));
                                coralArm = new CoralArm(new CoralArmIOReal(brakeSwitch));
                                transfer = new Transfer(new TransferIOReal());
                                VisionIOReal[] cameras = {
                                                new VisionIOReal("Left Front Camera",
                                                                Constants.VisionConstants.l_camera_transform),
                                                new VisionIOReal("Right Front Camera",
                                                                Constants.VisionConstants.r_camera_transform),
                                };

                                drive = new Swerve(new GyroIOPigeon(),
                                                new ModuleIOReal(0),
                                                new ModuleIOReal(1),
                                                new ModuleIOReal(2),
                                                new ModuleIOReal(3));
                                vision = new VisionSubsystem(drive::addVisionMeasurement, cameras);
                                riffCommands = new RiffCommands(elevator, coralArm, transfer);

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

                driverController.x().onTrue(TransferCommands.autoIntakeCoral(transfer));
                driverController.b().whileTrue(TransferCommands.riffOutake(transfer, coralArm));

                driverController.LB().whileTrue(new SwerveCommands.LocateToReefCommand(drive,
                                driverController,
                                true));
                driverController.RB().whileTrue(new SwerveCommands.LocateToReefCommand(drive,
                                driverController,
                                false));

                // LeftTrigger - slow
                // RightTrigger - fast

                // // intake, l2, l1
                // driverController.leftTrigger().whileTrue(TransferCommands.coralintake(transfer,
                // 5));
                // driverController.LB().whileTrue(TransferCommands.coralintake(transfer, 12));

                // // outake, l4, l3
                // driverController.rightTrigger().whileTrue(TransferCommands.coralintake(transfer,
                // -5));
                // driverController.RB().whileTrue(TransferCommands.coralintake(transfer, -12));

                /*----------------------------------------------------------------------------------------------------*/
                // operator:
                // open and close arm and elevator manual

                operatorController.rightTrigger().whileTrue(CoralArmCommands.setVoltage(coralArm, 1));
                operatorController.leftTrigger().whileTrue(CoralArmCommands.setVoltage(coralArm, -1));

                operatorController.RB().whileTrue(ElevatorCommands.setVoltage(elevator, 4));
                operatorController.LB().whileTrue(ElevatorCommands.setVoltage(elevator, -1));

                // open and close arm and elevator

                operatorController.PovUp().onTrue(ElevatorCommands.goToPosition(elevator, L4_ELEVATOR_POSITION));
                operatorController.PovDown().onTrue(ElevatorCommands.closeElevator(elevator));
                operatorController.PovLeft().onTrue(CoralArmCommands.goToPosition(coralArm, L4_ARM_POSITION));
                operatorController.PovRight().onTrue(CoralArmCommands.goToPosition(coralArm, CLOSE_ARM_POSITION));

                // coral intake position
                operatorController.leftStickClick().onTrue(riffCommands.coralIntakePos());

                operatorController.rightStickClick().onTrue(CoralArmCommands.closeArm(coralArm)
                                .alongWith(ElevatorCommands.closeElevator(elevator)));
                operatorController.y().onTrue(riffCommands.L4());
                operatorController.b().onTrue(riffCommands.L3());
                operatorController.x().onTrue(riffCommands.L2());
                operatorController.a().onTrue(riffCommands.L1());

                /*----------------------------------------------------------------------------------------------------*/
                // manuale:
                // // manuale elevator
                // manualController.rightTrigger().whileTrue(ElevatorCommands.openElevatorManual(elevator,
                // 5));
                // manualController.leftTrigger().whileTrue(ElevatorCommands.closeElevatorManual(elevator,
                // -1));
                // // elevator
                // manualController.a().onTrue(ElevatorCommands.closeElevator(elevator));
                // manualController.y().onTrue(ElevatorCommands.goToPosition(elevator,
                // L4_ELEVATOR_POSITION));

                // // manuale coral arm
                // manualController.PovLeft().whileTrue(CoralArmCommands.setVoltage(coralArm,
                // 1));
                // manualController.PovRight().whileTrue(CoralArmCommands.setVoltage(coralArm,
                // -1));
                // // coral arm
                // manualController.b().onTrue(CoralArmCommands.goToPosition(coralArm,
                // L4_ARM_POSITION));
                // manualController.x().onTrue(CoralArmCommands.goToPosition(coralArm,
                // CLOSE_ARM_POSITION));

                manualController.a().onTrue(CoralArmCommands.closeArm(coralArm));
                manualController.b().onTrue(CoralArmCommands.goToPosition(coralArm, L3_ARM_POSITION));
                manualController.x().onTrue(CoralArmCommands.goToPosition(coralArm, L2_ARM_POSITION));
                manualController.y().onTrue(CoralArmCommands.goToPosition(coralArm, 1.5));

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
