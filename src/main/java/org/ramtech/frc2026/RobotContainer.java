// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.ramtech.frc2026;

import static org.ramtech.frc2026.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.ramtech.frc2026.commands.DriveCommands;
import org.ramtech.frc2026.generated.TunerConstants;
import org.ramtech.frc2026.subsystems.drive.Drive;
import org.ramtech.frc2026.subsystems.drive.GyroIO;
import org.ramtech.frc2026.subsystems.drive.GyroIOPigeon2;
import org.ramtech.frc2026.subsystems.drive.ModuleIO;
import org.ramtech.frc2026.subsystems.drive.ModuleIOSim;
import org.ramtech.frc2026.subsystems.drive.ModuleIOTalonFX;
import org.ramtech.frc2026.subsystems.indexer.Indexer;
import org.ramtech.frc2026.subsystems.indexer.IndexerIO;
import org.ramtech.frc2026.subsystems.indexer.IndexerIOSim;
import org.ramtech.frc2026.subsystems.indexer.IndexerIOTalonFX;
import org.ramtech.frc2026.subsystems.shooter.flywheel.Flywheel;
import org.ramtech.frc2026.subsystems.shooter.flywheel.FlywheelIO;
import org.ramtech.frc2026.subsystems.shooter.flywheel.FlywheelIOSim;
import org.ramtech.frc2026.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import org.ramtech.frc2026.subsystems.shooter.hood.Hood;
import org.ramtech.frc2026.subsystems.shooter.hood.HoodIO;
import org.ramtech.frc2026.subsystems.shooter.hood.HoodIOSim;
import org.ramtech.frc2026.subsystems.shooter.hood.HoodIOTalonFX;
import org.ramtech.frc2026.subsystems.shooter.tower.Tower;
import org.ramtech.frc2026.subsystems.shooter.tower.TowerIO;
import org.ramtech.frc2026.subsystems.shooter.tower.TowerIOSim;
import org.ramtech.frc2026.subsystems.shooter.tower.TowerIOTalonFX;
// import org.ramtech.frc2026.subsystems.indexer.IndexerIOSim;
import org.ramtech.frc2026.subsystems.vision.Vision;
import org.ramtech.frc2026.subsystems.vision.VisionIO;
import org.ramtech.frc2026.subsystems.vision.VisionIOPhotonVision;
import org.ramtech.frc2026.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  @SuppressWarnings("unused")
  private final Drive drive;

  private final Vision vision;
  private final Indexer indexer;
  private final Tower tower;
  private final Hood hood;
  private final Flywheel flywheel;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(FLRamCam, robotToFL),
                new VisionIOPhotonVision(FRRamCam, robotToFR),
                new VisionIOPhotonVision(BLRamCam, robotToBL),
                new VisionIOPhotonVision(BRRamCam, robotToBR));
        indexer = new Indexer(new IndexerIOTalonFX());
        tower = new Tower(new TowerIOTalonFX());
        flywheel = new Flywheel(new FlywheelIOTalonFX());
        hood = new Hood(new HoodIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(FLRamCam, robotToFL, drive::getPose),
                new VisionIOPhotonVisionSim(FRRamCam, robotToFR, drive::getPose),
                new VisionIOPhotonVisionSim(BLRamCam, robotToBL, drive::getPose),
                new VisionIOPhotonVisionSim(BRRamCam, robotToBR, drive::getPose));
        indexer = new Indexer(new IndexerIOSim() {});
        tower = new Tower(new TowerIOSim());
        flywheel = new Flywheel(new FlywheelIOSim());
        hood = new Hood(new HoodIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        tower = new Tower(new TowerIO() {});
        indexer = new Indexer(new IndexerIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        hood = new Hood(new HoodIO() {});

        break;
    }
    RobotState.getInstance().setPoseSupplier(drive::getPose);
    RobotState.getInstance().setSpeedSupplier(drive::getChassisSpeeds);
    RobotState.getInstance().setModuleStateSupplier(drive::getModuleStates);

    // Continuously publish ShotCalculator parameters
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller
        .y()
        .onTrue(new InstantCommand(() -> indexer.setVoltages(6.0, 12.0)))
        .onTrue(new InstantCommand(() -> tower.setVoltage(12)))
        .onFalse(new InstantCommand(() -> indexer.stop()))
        .onFalse(new InstantCommand(() -> tower.stop()));
    controller
        .x()
        .onTrue(new InstantCommand(() -> indexer.setVoltages(6.0, 6.0)))
        .onTrue(new InstantCommand(() -> tower.setVoltage(6.0)))
        .onFalse(new InstantCommand(() -> indexer.stop()))
        .onFalse(new InstantCommand(() -> tower.stop()));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Pose2d getRobotPose() {
    return drive.getPose();
  }
}
