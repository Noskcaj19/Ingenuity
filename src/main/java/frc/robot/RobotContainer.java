// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DefaultTurret;
import frc.robot.commands.HomeElevator;
import frc.robot.commands.HomeExtender;
import frc.robot.commands.auto.Drive;
import frc.robot.commands.auto.DefaultAuto;
import frc.robot.commands.auto.FinalAuto;
import frc.robot.commands.limelight.LimelightHorizAim;
import frc.robot.subsystems.ClawSystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (i ncluding
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {

    // other
    private final Robot robot = new Robot();

    public final PneumaticsControlModule pCM = new PneumaticsControlModule(22);

    // Controllers
    private final XboxController primaryController = new XboxController(0);
    private final XboxController secondaryController = new XboxController(1);

    // Subsystems
    private final ClawSystem clawSystem = new ClawSystem(pCM);
    private final Drivetrain drivetrain = new Drivetrain();

    private final Limelight limelight = new Limelight();

    // commands
    private final DefaultDrive defaultDrive = new DefaultDrive(drivetrain, primaryController);
    private final DefaultTurret defaultTurret = new DefaultTurret(clawSystem, primaryController, secondaryController);
    private final DefaultAuto DefaultAuto = new DefaultAuto(drivetrain, clawSystem);
    private final Drive drive = new Drive(drivetrain, 1.3, 0.1);
    // private final FinalAuto finalAuto = new FinalAuto(drive, drivetrain);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        drivetrain.setDefaultCommand(defaultDrive);
        clawSystem.setDefaultCommand(defaultTurret);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        // new Trigger(m_exampleSubsystem::exampleCondition)
        // .onTrue(new ExampleCommand(m_exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is
        // pressed,
        // cancelling on release.
        // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
        // new JoystickButton(secondaryController, Button.kB.value)
        // .whileTrue(new LimelightHorizAim(clawSystem, limelight, secondaryController,
        // drivetrain));

        Shuffleboard.getTab("Debug").add(new HomeExtender(clawSystem));
        // Shuffleboard.getTab("Debug").add(new HomeElevator(clawSystem));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // write auto code once we get full robot
        return new FinalAuto(drive, drivetrain, clawSystem);
    }
}
