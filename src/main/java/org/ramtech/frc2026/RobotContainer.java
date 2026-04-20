package org.ramtech.frc2026;

import static org.ramtech.frc2026.subsystems.vision.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.ramtech.frc2026.RobotState.GlobalStates;
import org.ramtech.frc2026.commands.AutoSystemsCheck;
import org.ramtech.frc2026.commands.DriveCommands;
import org.ramtech.frc2026.commands.LowerIntake;
import org.ramtech.frc2026.generated.TunerConstants;
import org.ramtech.frc2026.subsystems.LedSubsystem;
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
import org.ramtech.frc2026.subsystems.intake.Intake;
import org.ramtech.frc2026.subsystems.intake.IntakeIO;
import org.ramtech.frc2026.subsystems.intake.IntakeIOSim;
import org.ramtech.frc2026.subsystems.intake.IntakeIOTalonFX;
import org.ramtech.frc2026.subsystems.shooter.ShotCalculator;
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
import org.ramtech.frc2026.subsystems.shooter.turret.Turret;
import org.ramtech.frc2026.subsystems.shooter.turret.TurretIO;
import org.ramtech.frc2026.subsystems.shooter.turret.TurretIOReal;
import org.ramtech.frc2026.subsystems.vision.Vision;
import org.ramtech.frc2026.subsystems.vision.VisionIO;
import org.ramtech.frc2026.subsystems.vision.VisionIOPhotonVision;
import org.ramtech.frc2026.subsystems.vision.VisionIOPhotonVisionSim;
import org.ramtech.frc2026.util.HubShiftUtil;
import org.ramtech.frc2026.util.HubShiftUtil.ShiftEnum;
import org.ramtech.frc2026.util.HubShiftUtil.ShiftInfo;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// Subsystems
	private final Drive drive;

	@SuppressWarnings("unused")
	private final Vision vision;

	private final Intake intake;
	private final Indexer indexer;
	private final Tower tower;
	private final Hood hood;
	private final Flywheel flywheel;
	private final Turret turret;
	private final LedSubsystem leds;

	// Controller
	private final CommandXboxController drivercontroller = new CommandXboxController(0);
	private final CommandXboxController operatorcontroller = new CommandXboxController(1);

	// Dashboard inputs
	private final LoggedDashboardChooser<Command> autoChooser;

	private double slowModeMultiplier = 1.0;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		switch (Constants.currentMode) {
			case REAL :
				// Real robot, instantiate hardware IO implementations
				// ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
				// a CANcoder
				drive = new Drive(new GyroIOPigeon2(), new ModuleIOTalonFX(TunerConstants.FrontLeft),
						new ModuleIOTalonFX(TunerConstants.FrontRight), new ModuleIOTalonFX(TunerConstants.BackLeft),
						new ModuleIOTalonFX(TunerConstants.BackRight));

				vision = new Vision(drive::addVisionMeasurement, new VisionIOPhotonVision(FLRamCam, robotToFL),
						new VisionIOPhotonVision(FRRamCam, robotToFR), new VisionIOPhotonVision(BLRamCam, robotToBL),
						new VisionIOPhotonVision(BRRamCam, robotToBR));
				intake = new Intake(new IntakeIOTalonFX());
				indexer = new Indexer(new IndexerIOTalonFX());
				tower = new Tower(new TowerIOTalonFX());
				flywheel = new Flywheel(new FlywheelIOTalonFX());
				hood = new Hood(new HoodIOTalonFX());
				turret = new Turret(new TurretIOReal());
				leds = new LedSubsystem();
				break;

			case SIM :
				// Sim robot, instantiate physics sim IO implementations
				drive = new Drive(new GyroIO() {
				}, new ModuleIOSim(TunerConstants.FrontLeft), new ModuleIOSim(TunerConstants.FrontRight),
						new ModuleIOSim(TunerConstants.BackLeft), new ModuleIOSim(TunerConstants.BackRight));

				vision = new Vision(drive::addVisionMeasurement,
						new VisionIOPhotonVisionSim(FLRamCam, robotToFL, drive::getPose),
						new VisionIOPhotonVisionSim(FRRamCam, robotToFR, drive::getPose),
						new VisionIOPhotonVisionSim(BLRamCam, robotToBL, drive::getPose),
						new VisionIOPhotonVisionSim(BRRamCam, robotToBR, drive::getPose));
				intake = new Intake(new IntakeIOSim());
				indexer = new Indexer(new IndexerIOSim() {
				});
				tower = new Tower(new TowerIOSim());
				flywheel = new Flywheel(new FlywheelIOSim());
				hood = new Hood(new HoodIOSim());
				turret = new Turret(new TurretIOReal());
				leds = new LedSubsystem();

				break;

			default :
				// Replayed robot, disable IO implementations
				drive = new Drive(new GyroIO() {
				}, new ModuleIO() {
				}, new ModuleIO() {
				}, new ModuleIO() {
				}, new ModuleIO() {
				});
				vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
				}, new VisionIO() {
				});
				intake = new Intake(new IntakeIO() {
				});
				indexer = new Indexer(new IndexerIO() {
				});
				tower = new Tower(new TowerIO() {
				});
				flywheel = new Flywheel(new FlywheelIO() {
				});
				turret = new Turret(new TurretIO() {
				});

				hood = new Hood(new HoodIO() {
				});
				leds = new LedSubsystem();

				break;
		}

		NamedCommands.registerCommand("lower_hood",
				new InstantCommand(() -> ShotCalculator.getInstance().setHoodUnsafe()));
		NamedCommands.registerCommand("shoot", shoot());

		NamedCommands.registerCommand("deploy_intake", deploy_intake());
		NamedCommands.registerCommand("raise_intake", raise_intake());

		NamedCommands.registerCommand("intake", intake_auto());

		RobotState.getInstance().setPoseSupplier(drive::getPose);
		RobotState.getInstance().setSpeedSupplier(drive::getChassisSpeeds);
		RobotState.getInstance().setModuleStateSupplier(drive::getModuleStates);
		RobotState.getInstance().setAccelerationSupplier(drive::getAcceleration);
		RobotState.getInstance().setFlywheelRpsSupplier(flywheel::getAverageVelocity);
		RobotState.getInstance().setTurretAngleSupplier(turret::getTurretAngle);
		RobotState.getInstance().setHoodAngleSupplier(hood::getHoodAngle);
		RobotState.getInstance().setGyroAngleRateSupplier(drive::getGyroAngleRate);

		// Set up auto routines
		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooserWithOptionsModifier(
				(stream) -> Constants.isComp ? stream.filter(auto -> auto.getName().startsWith("A")) : stream));

		// Set up SysId routines
		// autoChooser.addOption("Systems Check",
		// new timedcommand;

		autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
		autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
		autoChooser.addOption("Automatic Systems Check",
				new AutoSystemsCheck(drive, turret, hood, flywheel, intake, tower, indexer, leds));
		// autoChooser.addOption("Drive SysId (Quasistatic Forward)",
		// drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		// autoChooser.addOption("Drive SysId (Quasistatic Reverse)",
		// drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		// autoChooser.addOption("Drive SysId (Dynamic Forward)",
		// drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
		// autoChooser.addOption("Drive SysId (Dynamic Reverse)",
		// drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

		// Configure the button bindings
		configureButtonBindings();

		new Trigger(() -> {
			ShiftInfo info = HubShiftUtil.getShiftedShiftInfo();
			double remainingTime = info.remainingTime();
			return remainingTime <= 5 && remainingTime >= 4.5 && info.currentShift() != ShiftEnum.DISABLED;
		}).whileTrue(
				// Command to run while the trigger is true, and what to do when it ends
				Commands.startEnd(() -> drivercontroller.setRumble(RumbleType.kBothRumble, 1.0), // Start rumble
						() -> drivercontroller.setRumble(RumbleType.kBothRumble, 0.0) // Stop rumble
				));

	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		/*
		 * Driving
		 */
		// Drive
		drive.setDefaultCommand(DriveCommands.joystickDrive(drive,
				() -> drive.clampToMaxJoystick(-drivercontroller.getLeftY() * slowModeMultiplier),
				() -> drive.clampToMaxJoystick(-drivercontroller.getLeftX() * slowModeMultiplier),
				() -> drive.clampToMaxJoystick(-drivercontroller.getRightX()),
				() -> drive.getTranslationalSlewFromState(RobotState.getInstance().getGlobalState()),
				() -> drive.getRotationalSlewFromState(RobotState.getInstance().getGlobalState())));
		// Reset gyro
		drivercontroller.start().onTrue(Commands
				.runOnce(() -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)), drive)
				.ignoringDisable(true));
		// X pattern
		drivercontroller.povDown().onTrue(Commands.runOnce(drive::stopWithX, drive)); // x pattern
		// Slow Mode
		drivercontroller.rightBumper().whileTrue(slowMode());

		/*
		 * Intaking
		 */
		// Intake
		drivercontroller.leftTrigger().whileTrue(intake());
		drivercontroller.rightTrigger().whileTrue(outtake());

		// Raise Intake
		drivercontroller.y().onTrue(raise_intake());

		/*
		 * Shooting
		 */
		// Re-enable shooter
		drivercontroller.leftBumper().onTrue(new InstantCommand(() -> ShotCalculator.getInstance().requestSafe()));

		// Shoot
		drivercontroller.leftBumper().and(() -> ShotCalculator.getInstance().getLatest().hoodSafe())
				.and(() -> ShotCalculator.getInstance().getLatest().shootingAllowed()).whileTrue(shoot());
		drivercontroller.b().whileTrue(shoot());

		// Overrides for helping battery conservation during auto testing.
		drivercontroller.a().onTrue(conserve());

		/*
		 * Operator Overrides
		 */
		// Lock Turret
		operatorcontroller.leftBumper().onTrue(new InstantCommand(() -> turret.setTurretDriverLock(true)));
		operatorcontroller.rightBumper().onTrue(new InstantCommand(() -> turret.setTurretDriverLock(false)));

		operatorcontroller.a().onTrue(new InstantCommand(() -> hood.setHoodDriverLock(true)));
		operatorcontroller.y().onTrue(new InstantCommand(() -> hood.setHoodDriverLock(false)));

		// drivercontroller.povDown().onTrue(new InstantCommand(() ->
		// flywheel.enableCalculation()));

		// Raise Rps
		// operatorcontroller.povUp().onTrue(Commands.sequence(Commands.runOnce(() -> {
		// double currentBump = Preferences.getDouble("rpsBump", 0.0);
		// double newBump = currentBump + 0.5;

		// Preferences.setDouble("rpsBump", newBump);

		// System.out.println("Shooter RPM bumped to: " + newBump);

		// }), new InstantCommand(() ->
		// ShotCalculator.getInstance().refreshRpsBump())));

		// Lower Rps
		// operatorcontroller.povDown().onTrue(Commands.sequence(Commands.runOnce(() ->
		// {
		// double currentBump = Preferences.getDouble("rpsBump", 0.0);
		// Preferences.setDouble("rpsBump", currentBump - 0.5);
		// System.out.println("Shooter RPM dropped to: " + (currentBump - 0.5));
		// }), new InstantCommand(() ->
		// ShotCalculator.getInstance().refreshRpsBump())));

		DriverStation.silenceJoystickConnectionWarning(true);
	}

	// /**
	// * Use this to pass the autonomous command to the main {@link Robot} class.
	// *
	// * @return the command to run in autonomous
	// */

	public Command getAutonomousCommand() {
		return autoChooser.get();
	}

	public void updateAutoTrajectoryPreview() {
		// 1. You need the string name of the auto.
		// (If using a custom String chooser, get it here. For example: String
		// selectedAuto = myStringChooser.get();)
		String selectedAuto = autoChooser.get().getName(); // Replace with your chooser's string output

		if (selectedAuto != null && !selectedAuto.isEmpty()) {
			try {
				// 2. Load the path group from the auto file
				List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(selectedAuto);
				List<Pose2d> pathPoses = new ArrayList<>();

				// 3. Extract all Pose2d points from each path in the auto
				for (PathPlannerPath path : paths) {
					pathPoses.addAll(path.getPathPoses());
				}
				// 4. Send to RobotState
				RobotState.getInstance().setPreviewPathPlannerTrajectory(pathPoses.toArray(new Pose2d[0]));
			} catch (Exception e) {
				// Failsafe in case the auto file doesn't exist or is purely command-based
				// without paths
				RobotState.getInstance().setPreviewPathPlannerTrajectory(new Pose2d[0]);
			}
		}
	}

	public Command slowMode() {
		return new StartEndCommand(() -> slowModeMultiplier = 0.8, () -> slowModeMultiplier = 1);
	}

	public Command shoot() {
		return Commands.run(() -> {
			// This runs continuously, checking the condition every 20ms loop
			if (!ShotCalculator.getInstance().transitionInProgress) {
				if (RobotState.getInstance().getGlobalState() != GlobalStates.SHOOTING) {
					RobotState.getInstance().setGlobalState(GlobalStates.SHOOTING);
				}

				// tower.setVoltage(10);
				// indexer.setVoltage(10);
				// flywheel.enableCalculation();
				intake.setRollerVoltage(13);
				tower.setVoltage(13);
				indexer.setVoltage(13);
				// intake.setAutoMode(IntakeIOAutoDirections.FORWARD);
			} else {
				tower.stop();
				indexer.setVoltage(-13);
				intake.setRollerVoltage(13);

			}
		}).alongWith(Commands.startEnd(() -> {
			ShotCalculator.getInstance().requestSafe();
			flywheel.enableCalculation();
		}, () -> {
			indexer.stop();
			tower.stop();
			intake.stopRollers();
			if (RobotState.getInstance().getGlobalState() == GlobalStates.SHOOTING) {
				RobotState.getInstance().setGlobalState(GlobalStates.IDLE);

			}
		}, flywheel, indexer, tower));
	}

	public Command deploy_intake() {
		return (Commands.runOnce(() -> intake.lowerPivot()));
	}

	public Command raise_intake() {
		return Commands.runOnce(() -> intake.setPivotPosition(0.185));
	}

	public Command intake() {
		return Commands.either(new LowerIntake(intake, turret), // Run this if true
				Commands.none(), // Do nothing if false
				turret::isIntakeLocked).alongWith(Commands.startEnd(() -> {
					if (RobotState.getInstance().getGlobalState() != GlobalStates.SHOOTING
							&& RobotState.getInstance().getGlobalState() != GlobalStates.INTAKING) {
						RobotState.getInstance().setGlobalState(GlobalStates.INTAKING);
					}
					intake.lowerPivot();
					intake.setRollerVoltage(13);
					indexer.setVoltage(-13);
					// intake.setRollerVoltage(10.0);
					// indexer.setVoltage(-10);
				}, () -> {
					intake.stopRollers();
					indexer.stop();
					if (RobotState.getInstance().getGlobalState() == GlobalStates.INTAKING) {
						RobotState.getInstance().setGlobalState(GlobalStates.IDLE);
					}
				}, indexer));
	}

	public Command outtake() {
		return Commands.either(new LowerIntake(intake, turret), // Run this if true
				Commands.none(), // Do nothing if false
				turret::isIntakeLocked).alongWith(Commands.startEnd(() -> {
					intake.lowerPivot();
					intake.setRollerVoltage(-13);
					indexer.setVoltage(-13);
				}, () -> {
					intake.stopRollers();
					indexer.stop();
				}, indexer));
	}

	public Command intake_auto() {
		return Commands.either(new LowerIntake(intake, turret), // Run this if true
				Commands.none(), // Do nothing if false
				turret::isIntakeLocked).alongWith(Commands.startEnd(() -> {
					intake.lowerPivot();
					indexer.stop();
					intake.setRollerVoltage(13.0);
				}, () -> {
					intake.stopRollers();
					indexer.stop();
				}, indexer));
	}

	public Command conserve() {
		return Commands.runOnce(() -> {
			indexer.stop();;
			intake.stopRollers();
			tower.stop();
			flywheel.stop();
		}, indexer, intake, flywheel);
	}

}
