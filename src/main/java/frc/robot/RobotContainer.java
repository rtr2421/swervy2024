// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.RunIntakeWithDelay;
import frc.robot.commands.ShootNote;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

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
  private Boolean fieldOrientedMode = true;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController helperXbox = new CommandXboxController(OperatorConstants.kHelperControllerPort);

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
        () -> {
          return -driverXbox.getRightX() + driverXbox.getLeftTriggerAxis() - driverXbox.getRightTriggerAxis();
        },
        () -> fieldOrientedMode);

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

    SmartDashboard.putNumber("ShooterSpeed1", 1000);
    SmartDashboard.putNumber("ShooterSpeed2", 1000);
    SmartDashboard.putData("SetShooterVelocity", new StartEndCommand(
        () -> shooter.setVelocity(SmartDashboard.getNumber("ShooterSpeed1", 1000),
            SmartDashboard.getNumber("ShooterSpeed2", 1000)),
        () -> shooter.stop(), shooter));
    SmartDashboard.putNumber("ShooterP", 0.001);
    SmartDashboard.putData("SetShooterP", new InstantCommand(
        () -> shooter.setP(SmartDashboard.getNumber("ShooterP", 0.001))));

    SmartDashboard.putData("Reset Pose", new InstantCommand(() -> resetPoseAngle()));
    SmartDashboard.putData("Override Climber Safety",
        new StartEndCommand(() -> climber.setSafety(false), () -> climber.setSafety(true)));

    commonBindings(driverXbox);
    commonBindings(helperXbox);

  }

  private void commonBindings(CommandXboxController joystick) {
    joystick.a().toggleOnTrue(
        new RunIntakeWithDelay(indexer, intake)
            .andThen(new InstantCommand(() -> joystick.getHID().setRumble(RumbleType.kBothRumble, 1)))
            .andThen(new WaitCommand(1))
            .andThen(new InstantCommand(() -> joystick.getHID().setRumble(RumbleType.kBothRumble, 0))));

    joystick.start().onTrue(new InstantCommand(() -> {
      fieldOrientedMode = !fieldOrientedMode;
    }));

    joystick.x().onTrue(new ShootNote(shooter, indexer, true)
        .andThen(new InstantCommand(() -> {
          shooter.stop();
          indexer.stop();
        })));
    joystick.b().onTrue(new ShootNote(shooter, indexer, false)
        .andThen(new InstantCommand(() -> {
          shooter.stop();
          indexer.stop();
        })));

    joystick.povDown().whileTrue(new StartEndCommand(() -> climber.retract(), () -> climber.stop(), climber));
    joystick.povUp().whileTrue(new StartEndCommand(() -> climber.extend(), () -> climber.stop(), climber));

    // driverXbox.y().toggleOnTrue(new RunClimber(climber));

    joystick
        .y()
        .whileTrue(new InstantCommand(() -> {
          indexer.reverse();
          intake.reverse();
        }, intake))
        .onFalse(new InstantCommand(() -> {
          indexer.stop();
          intake.stop();
        }, intake));
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
    NamedCommands.registerCommand("shoot", new ShootNote(shooter, indexer, true)
        .andThen(new InstantCommand(() -> {
          shooter.stop();
          indexer.stop();
        })));
    NamedCommands.registerCommand("RunIntake", new RunIntakeWithDelay(indexer, intake));
    NamedCommands.registerCommand("RunIntakeAndShoot", new RunCommand(() -> {
      intake.start();
      indexer.startIntaking();
      shooter.highShot();
    }, intake, indexer, shooter));

    // autonomousChooser.addOption("Right 3 shot", Autos.rightShoot3Shots());

    autonomousChooser.setDefaultOption("Drive forward", Autos.driveForward(drive));
    autonomousChooser.addOption("Small side 1 shot", Autos.simpleAutoSmall());
    autonomousChooser.addOption("Big side 1 shot", Autos.simpleAutoBig());

    autonomousChooser.addOption("Shoot 2 from Center", Autos.centerShoot2Close());
    autonomousChooser.addOption("Shoot 3 from Center (should do)", Autos.centerShoot3Far());

    autonomousChooser.addOption("Shoot 2 from Big side", Autos.bigShoot2Close());
    autonomousChooser.addOption("Shoot 3 from Big side (should do)", Autos.bigShoot3Far());
    autonomousChooser.addOption("Shoot 2 farthest notes from big side", Autos.bigShoot3Far2());
    autonomousChooser.addOption("bigMoveCenterNotes", Autos.bigMoveCenterNotes());

    autonomousChooser.addOption("Shoot 3 from Small side (should do)", Autos.smallShoot3Far());
    autonomousChooser.addOption("Shoot 2 from Small side", Autos.smallShoot2Close());
    autonomousChooser.addOption("Disrupt Center From Small side", Autos.smallDisruptCenter());

    autonomousChooser.addOption("Centered Shoot 2 Non path planner",
        Autos.shoot2Pieces(drive, shooter, intake, indexer));
    SmartDashboard.putData("auto choices", autonomousChooser);
  }

  private void resetPoseAngle() {
    var currentPose = drive.getPose();
    var newPose = new Pose2d(currentPose.getTranslation(), new Rotation2d(0));
    drive.resetOdometry(newPose);
  }

}
