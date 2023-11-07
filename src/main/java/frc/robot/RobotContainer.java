// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOFalcon;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;

import frc.robot.subsystems.slider.Slider;
import frc.robot.subsystems.slider.SliderIO;
import frc.robot.subsystems.slider.SliderIOSim;
import frc.robot.subsystems.slider.SliderIOSparkMax;

import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.GyroIONavx;
import frc.robot.subsystems.drive.GyroIOPigeon;

import frc.robot.subsystems.drive.LimelightIO;
import frc.robot.subsystems.drive.LimelightIOSim;
import frc.robot.subsystems.drive.LimelightIONetwork;

//Commands:
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ElevatorGoToPosition;
import frc.robot.commands.SliderGoToPosition;

//Autos
import frc.robot.autos.Example_Auto;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private final double allowableElevatorTeleopInch = Constants.ElevatorSubsystem.allowableTeleopErrorInch;
  private final double allowableSliderTeleopInch = Constants.ElevatorSubsystem.allowableTeleopErrorInch;
  private double invertJoystick;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices:");

  // Controllers

  // Driver control 1
  private final Joystick driver_1 = new Joystick(0);
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton lockToHeading = new JoystickButton(driver_1,
      XboxController.Button.kA.value);
  private final JoystickButton zeroGyro = new JoystickButton(driver_1,
      XboxController.Button.kY.value);
  private final JoystickButton calibrate = new JoystickButton(driver_1,
      XboxController.Button.kRightBumper.value);

  // /* Driver Buttons */
  // private final Joystick driver = new Joystick(0);
  // private final int translationAxis = Joystick.AxisType.kY.value;
  // private final int strafeAxis = Joystick.AxisType.kX.value;
  // private final int rotationAxis = Joystick.AxisType.kZ.value;
  // // private final int throttleAxis = Joystick.AxisType.kTwist.value;
  // private final JoystickButton calibrate = new JoystickButton(driver, 12);

  // private final JoystickButton zeroGyro = new JoystickButton(driver, 8);
  // // private final JoystickButton robotCentric = new JoystickButton(driver, 7);
  // private final JoystickButton lockToHeading = new JoystickButton(driver,
  // 1);

  // /* Driver control 2 */
  private final Joystick driver_2 = new Joystick(1);
  private final JoystickButton flipIntakeMode = new JoystickButton(driver_2, 7);
  private final JoystickButton intakeIn = new JoystickButton(driver_2, XboxController.Button.kRightBumper.value);
  private final JoystickButton intakeOut = new JoystickButton(driver_2, XboxController.Button.kLeftBumper.value);

  // Buttons for elevator
  private final JoystickButton elevatorBottom = new JoystickButton(driver_2, XboxController.Button.kA.value);
  private final JoystickButton elevatorMid = new JoystickButton(driver_2, XboxController.Button.kX.value);
  private final JoystickButton elevatorLoading = new JoystickButton(driver_2, XboxController.Button.kB.value);
  private final JoystickButton elevatorTop = new JoystickButton(driver_2, XboxController.Button.kY.value);

  // DPad for slider
  private final POVButton sliderIn = new POVButton(driver_2, 180);
  private final POVButton sliderOut = new POVButton(driver_2, 0);

  // Subsystem setup
  private final Swerve j_Swerve;
  private final LimelightIO limelight;
  private final GyroIO gyro;
  private final Intake intake;
  private final Elevator elevator;
  private final Slider slider;

  public RobotContainer() {

    if (Constants.enableLimelight) {
      limelight = new LimelightIONetwork();
    } else {
      limelight = new LimelightIOSim();
    }
    if (ConstantsSwerve.usingPigeon) {
      gyro = new GyroIOPigeon();
    } else {
      gyro = new GyroIONavx();
    }
    switch (Constants.getMode()) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        Timer.delay(1.0);
        j_Swerve = new Swerve(
            limelight,
            gyro,
            new ModuleIOFalcon(0, ConstantsSwerve.Mod0.constants),
            new ModuleIOFalcon(1, ConstantsSwerve.Mod1.constants),
            new ModuleIOFalcon(2, ConstantsSwerve.Mod2.constants),
            new ModuleIOFalcon(3, ConstantsSwerve.Mod3.constants));
        if (Constants.chassisOnly) {
          intake = new Intake(new IntakeIO() {
          });
          elevator = new Elevator(new ElevatorIO() {
          });
          slider = new Slider(new SliderIO() {
          });
        } else {
          intake = new Intake(new IntakeIOSparkMax());
          elevator = new Elevator(new ElevatorIOSparkMax());
          slider = new Slider(new SliderIOSparkMax());
        }
        break;
      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        j_Swerve = new Swerve(
            new LimelightIOSim(),
            new GyroIOSim(() -> -driver_1.getRawAxis(rotationAxis)),
            new ModuleIOSim(0, ConstantsSwerve.Mod0.constants),
            new ModuleIOSim(1, ConstantsSwerve.Mod1.constants),
            new ModuleIOSim(2, ConstantsSwerve.Mod2.constants),
            new ModuleIOSim(3, ConstantsSwerve.Mod3.constants));
        intake = new Intake(new IntakeIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        slider = new Slider(new SliderIOSim());
        break;

      // Replayed robot, disable IO implementations
      default:
        j_Swerve = new Swerve(
            new LimelightIO() {
            },
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
        intake = new Intake(new IntakeIO() {
        });
        elevator = new Elevator(new ElevatorIO() {
        });
        slider = new Slider(new SliderIO() {
        });
        break;
    }

    // Auto chooser setup
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Example_Auto", new Example_Auto(j_Swerve, intake, elevator, slider));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Set default command
    invertJoystick = Constants.getMode() == Constants.Mode.SIM ? 1.0 : -1.0;
    j_Swerve.setDefaultCommand(new TeleopSwerve(
        j_Swerve,
        () -> driver_1.getRawAxis(translationAxis),
        () -> driver_1.getRawAxis(strafeAxis),
        () -> invertJoystick * driver_1.getRawAxis(rotationAxis),
        () -> lockToHeading.getAsBoolean()));

    // Sensors + Modes
    zeroGyro.onTrue(new InstantCommand(() -> j_Swerve.zeroGyro()));
    calibrate.onTrue(new InstantCommand(() -> j_Swerve.calibrateGyro()));
    flipIntakeMode.onTrue(new InstantCommand(() -> intake.flipIntakeMode()));

    // Intake
    intakeIn.whileTrue(new StartEndCommand(() -> intake.intakeIn(), () -> intake.holdCurrent(), intake));
    intakeOut.whileTrue(new StartEndCommand(() -> intake.intakeOut(), intake::stop, intake));

    // Elevator
    elevatorBottom.onTrue(
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosBottom, allowableElevatorTeleopInch, elevator));
    elevatorMid.onTrue(
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosMid, allowableElevatorTeleopInch, elevator));
    elevatorLoading.onTrue(new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosLoading,
        allowableElevatorTeleopInch, elevator));
    elevatorTop.onTrue(
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosTop, allowableElevatorTeleopInch, elevator));

    // Slider
    sliderIn.onTrue(new SliderGoToPosition(Constants.SliderSubsystem.sliderIn, allowableSliderTeleopInch, slider));
    sliderOut.onTrue(new SliderGoToPosition(Constants.SliderSubsystem.sliderOut, allowableSliderTeleopInch, slider));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetElevator() {
    elevator.setPositionSetPoint(elevator.getPosition());
  }
}
