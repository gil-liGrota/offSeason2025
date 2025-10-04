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

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.POM_lib.Joysticks.PomXboxController;
import frc.robot.POM_lib.sensors.POMDigitalInput;
import frc.robot.commands.AutonomousRoutines;
import frc.robot.commands.CoralArmCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.LEDsCommands;
import frc.robot.commands.RiffCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.commands.TransferCommands;
import frc.robot.subsystems.CoralArm.CoralArm;
import frc.robot.subsystems.CoralArm.CoralArmIOReal;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorReal;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.LEDs.LEDsIOReal;
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
        // Controller
        private final CommandPS5Controller driverController = new CommandPS5Controller(0);
        private final PomXboxController operatorController = new PomXboxController(1);
        private final PomXboxController manualController = new PomXboxController(2);

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        private boolean isRelative;
        private POMDigitalInput brakeSwitch = new POMDigitalInput(BRAKE_SWITCH);
        RiffCommands riffCommands;

        private SwerveDriveSimulation driveSimulation = null;

        Elevator elevator;
        CoralArm coralArm;
        Transfer transfer;
        Swerve drive;
        VisionSubsystem vision;
        LEDs leds;

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

                                leds = new LEDs(new LEDsIOReal());

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
                autoChooser.addDefaultOption("otonomi",
                                AutonomousRoutines.putReef(4, drive, elevator, coralArm, transfer, false));

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
                                                () -> driverController.getLeftY() * -0.75,
                                                () -> driverController.getLeftX() * -0.75,
                                                () -> driverController.getRightX() * -0.75));

                driverController.triangle().onTrue(drive.resetGyroCommand());

                driverController.square().onTrue(TransferCommands.autoIntakeCoral(transfer));

                driverController.R1().whileTrue(new SwerveCommands.LocateToReefCommand(drive,
                                driverController,
                                false));
                driverController.L1().whileTrue(new SwerveCommands.LocateToReefCommand(drive,
                                driverController,
                                true));
                driverController.L1().or(driverController.R1()).onFalse(new InstantCommand(drive::stop, drive));

                driverController.circle().whileTrue(TransferCommands.riffOutake(transfer, coralArm));
                // LeftTrigger - slow
                driverController.L2().whileTrue(SwerveCommands.joystickDrive(
                                drive,
                                () -> driverController.getLeftY() * -0.4,
                                () -> driverController.getLeftX() * -0.4,
                                () -> driverController.getRightX() * -0.4));
                driverController.R2().whileTrue(SwerveCommands.joystickDrive(
                                drive,
                                () -> driverController.getLeftY() * -1,
                                () -> driverController.getLeftX() * -1,
                                () -> driverController.getRightX() * -1));

                // driverController.cross().whileTrue(AutonomousRoutines.driveToPoseInCorrectAlliance(drive,
                // new Pose2d(16.6, 0.85, Rotation2d.fromDegrees(125)),
                // false));

                driverController.cross().whileTrue(SwerveCommands.driveForwardSlowRight(drive));

                /*----------------------------------------------------------------------------------------------------*/
                // operator:
                // open and close arm and elevator manual

                operatorController.rightTrigger().whileTrue(CoralArmCommands.setVoltage(coralArm, 1));
                operatorController.leftTrigger().whileTrue(CoralArmCommands.setVoltage(coralArm, -1));

                operatorController.RB().whileTrue(ElevatorCommands.setVoltage(elevator, 4));
                operatorController.LB().whileTrue(ElevatorCommands.setVoltage(elevator, -1));

                // outake
                operatorController.PovLeft().whileTrue(TransferCommands.coralintake(transfer, -12));
                operatorController.PovRight().whileTrue(TransferCommands.coralintake(transfer, 3));

                // coral intake position
                operatorController.PovUp().onTrue(riffCommands.coralIntakePos());

                operatorController.PovDown().onTrue(riffCommands.closeAll());

                operatorController.y().onTrue(riffCommands.L4());
                operatorController.b().onTrue(riffCommands.L3());
                operatorController.x().onTrue(riffCommands.L2());
                operatorController.a().onTrue(riffCommands.L1());

                driverController.povUp().whileTrue(LEDsCommands.setAll(leds, Color.kPurple));
                driverController.povDown().whileTrue(LEDsCommands.setAll(leds, Color.kBlack));

        }

        public Command pathPlanerCommand() {
                try {
                        // Load the path you want to follow using its name in the GUI
                        PathPlannerPath path = PathPlannerPath.fromPathFile("reefToStation");

                        // Create a path following command using AutoBuilder. This will also trigger
                        // event markers.
                        return AutoBuilder.followPath(path);
                } catch (Exception e) {
                        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                        return Commands.none();
                }
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
