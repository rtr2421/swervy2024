// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShootNote;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Climber;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final SwerveSubsystem drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Indexer indexer = new Indexer();
  private final Climber climber = new Climber();
  private Boolean driveMode = false;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private SendableChooser<Command> autonomousChooser = new SendableChooser<>();

 private final Field2d field;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureAutos();
    field = new Field2d();
        SmartDashboard.putData("Field", field);

    
    TeleopDrive teleopDrive = new TeleopDrive(drive,
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> driveMode);

    drive.setDefaultCommand(teleopDrive);

         // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
            SmartDashboard.putNumber("CurrentX", pose.getX());
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
            SmartDashboard.putNumber("TargetX", pose.getX());
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });
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
    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

    driverXbox.a().toggleOnTrue(new RunIntake(indexer, intake));
    driverXbox.start().onTrue(new InstantCommand(() -> {driveMode = !driveMode; SmartDashboard.putBoolean("Drive Mode", driveMode);}));
    driverXbox.b().whileTrue(new InstantCommand(()-> shooter.lowShot())
        .andThen(new WaitUntilCommand(shooter::atSpeed))
        .andThen(new RunCommand(()-> indexer.start())))
        .onFalse(new InstantCommand(()-> {shooter.stop(); indexer.stop();}));
    driverXbox.x().whileTrue(new ShootNote(shooter, indexer, true))
        .onFalse(new InstantCommand(()-> {shooter.stop(); indexer.stop();}));
    driverXbox.y().toggleOnTrue(new RunClimber(climber));

    
    driverXbox
        .leftBumper()
        .whileTrue(new InstantCommand(() -> intake.reverse(), intake))
        .onFalse(new InstantCommand(()-> intake.stop(), intake)); 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autonomousChooser.getSelected();
  }

  private void configureAutos() {
    NamedCommands.registerCommand("shoot", new ShootNote(shooter, indexer, true).withTimeout(2)
      .andThen(new InstantCommand(()-> {shooter.stop(); indexer.stop();})));
    // TODO: Change timeout on above line
    NamedCommands.registerCommand("RunIntake", new RunIntake(indexer, intake));
    

   
    autonomousChooser.setDefaultOption("Drive forward", Autos.driveForward(drive));
    autonomousChooser.addOption("Shoot 2 pieces", Autos.shoot2Pieces(drive, shooter, intake, indexer));
    autonomousChooser.addOption("Out Alliance area", Autos.outAllianceArea(drive));
    autonomousChooser.addOption("Shoot one amp", Autos.singleAmp(drive, indexer, shooter));
    autonomousChooser.addOption("Test auto path planner", Autos.testAuto());
    autonomousChooser.addOption("2 shot auto", Autos.shoot2HighShots());
    autonomousChooser.addOption("centerFarHoop", Autos.centerFarHoop());
    autonomousChooser.addOption("leftShoot3Shots", Autos.leftShoot3Shots());
    autonomousChooser.addOption("Shoot 2 amp", Autos.doubleAmp(drive, indexer, shooter, intake));
    
    autonomousChooser.addOption("Sean", Autos.sean());
    autonomousChooser.addOption("dumb", Autos.dumbAuto(drive));
    SmartDashboard.putData("auto choices", autonomousChooser);
  }
}
