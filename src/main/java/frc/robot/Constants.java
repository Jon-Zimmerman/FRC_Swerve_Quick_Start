package frc.robot;

//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.auto.PIDConstants;

//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.math.util.Units;
//import frc.lib.util.COTSFalconSwerveConstants;
//import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.wpilibj.RobotBase;

import frc.lib.util.Alert;
import frc.lib.util.Alert.AlertType;

public final class Constants {
    // public static final Mode currentMode = Mode.REAL;
    private static final RobotType robot = RobotType.SIM;
    public static final boolean enableLimelight = false;
    public static final boolean chassisOnly = false;
    public static final boolean LimelightAffectsOdometry = false;
    public static final boolean enableLockWheelsAt45 = false; // Not currently implemented at all, value does nothing
    public static final boolean enableLockToHeading = false;
    public static final double acceptableLimelightMergeDistMeters = 3; // distance from back of grid in X direction to
                                                                       // allow tag inputs

    public static final double simLoopPeriodSecs = 0.02;
    public static final boolean tuningMode = false;

    public static final double translationStickDeadband = 0.08;
    public static final double rotationStickDeadband = 0.08;

    public static final class AutoConstants {
        public static final boolean readAllianceColortoFlipPaths = true;

        public static final double kMaxSpeedMetersPerSecond = 1.8;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = 0.1 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 0.1 * Math.PI;

        public static final PIDConstants translationPIDConstants = new PIDConstants(0.1, 0.0, 0.0);
        public static final PIDConstants rotationPIDConstants = new PIDConstants(0.7, 0.0, 0.0);

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    // Elevator Extender Motor
    public static final class ElevatorSubsystem {
        public static final int deviceID = 13;
        public static final boolean isInverted = true;

        // FeedForward Control
        public static final double ks = 0.00;
        public static final double kv = 0.04; // 0.2
        public static final double kg = 0.5; // 0.75

        // public static final double ks = 0.00;
        // public static final double kv = 0.25;
        // public static final double kg = 0.85;

        public static final double kP = 0.05;
        public static final double kI = 0.00;
        public static final double kD = 0.0;
        public static final double kIz = 0;
        public static final double kFF = 0;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;

        public static final double gearRatio = 25.0;
        public static final double sprocketDiameterInch = 1.92;

        // motor shaft details
        public static final int maxCurrentAmps = 30;
        public static final double maxAngularVelocityRPM = 100.0;
        public static final double maxAngularAccRPMPerSec = 80.0;
        public static final double minOutputVelocityRPM = 10.0; // requests below this no voltage output
        public static final double allowableSmartMotionPosErrorRotations = 3.0 * gearRatio;
        public static final double autoPositionErrorInch = 2.0;

        // Elevator details
        public static final double maxLinearVelocityInchPerSec = 28.0;
        public static final double maxLinearAccelerationInchPerSec = 28.0;

        // Inches
        public static final double elevatorSoftLimitLowerInch = 0;
        public static final double elevatorPosBottom = 0.0;
        public static final double elevatorPosMid = 17.2;
        public static final double elevatorPosAltLoading = 22.0;
        public static final double elevatorPosLoading = 23.0;
        public static final double elevatorPosTop = 24.2;
        public static final double elevatorSoftLimitUpperInch = 25.25;

        public static final double simCarriageWeightKg = 9.0; // ~20 lbs
        public static final double allowableTeleopErrorInch = 1.0;
    }

    // Slider Motor
    public static final class SliderSubsystem {
        public static final int deviceID = 14;
        public static final int sensorResolution = 2048;
        public static final boolean isInverted = false;
        // FeedForward Control
        public static final double ks = 0.0;
        public static final double kv = 0.00;
        public static final double kg = 0.00;

        public static final double gearRatio = 4.0 * 30.0 / 12.0;
        public static final double sprocketDiameterInch = 1.29;
        public static final double kP = 0.06;
        public static final double kI = 0.00;
        public static final double kD = 0.0;
        public static final double kIz = 0.0;
        public static final double kFF = 0.0;
        public static final double kMaxOutput = 1.0;
        public static final double kMinOutput = -1.0;
        public static final int kTimeoutMs = 30;

        public static final int maxCurrentAmps = 30;

        public static final double maxAngularVelocityRPM = 1200.0;
        public static final double maxAngularAccRPMPerSec = 1500.0;
        public static final double minOutputVelocityRPM = 20.0; // requests below this no voltage output
        public static final double allowableSmartMotionPosErrorRotations = 1.4 * gearRatio;
        public static final double autoPositionErrorInch = 2.0;

        public static final double maxLinearVelocityInchPerSec = 60;
        public static final double maxLinearAccelerationInchPerSec = 70;
        // Inches
        public static final double sliderSoftLimitLowerInch = 0.0;
        public static final double sliderIn = 0.0;
        public static final double sliderOut = 14.6;
        public static final double sliderSoftLimitUpperInch = 15.0;

        public static final double simCarriageWeightKg = 4.0; // ~20 lbs

    }

    // Intake motor
    public static final class IntakeSubsystem {
        public static final int deviceID = 12;
        public static final boolean isInverted = false;

        // FeedForward Control
        public static final double ks = 0.0;
        public static final double kv = 0.000;
        // Closed Loop Control
        public static final double kP = 0.1;
        public static final double kI = 0.00;
        public static final double kD = 0.0;
        public static final double kIz = 0;
        public static final double kFF = 0;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;

        public static final double gearRatio = 4.0 * 2.0;

        public static final int maxCurrentAmps = 25;
        public static final int holdCubeCurrentAmps = 10;
        public static final int holdConeCurrentAmps = 10;
        // Velocity control mode
        public static final double intakeInCubeVelRPM = 50.0;

        public static final double intakeOutCubeVelRPM = -50.0;

        public static final double intakeInConeVelRPM = -100.0;

        public static final double intakeOutConeVelRPM = 100.0;
        // Voltage control mode
        public static final double holdCubeVoltage = 4.0;
        public static final double holdConeVoltage = -3.2;

        public static final double intakeInCubeVoltage = 6.0;
        public static final double intakeOutCubeVoltage = -5.0;

        public static final double intakeInConeVoltage = -7.0;
        public static final double intakeOutConeVoltage = 5.0;

    }

    private static final Alert invalidRobotAlert = new Alert(
            "Invalid robot selected, using competition robot as default.",
            AlertType.ERROR);

    public static RobotType getRobot() {
        if (RobotBase.isReal()) {
            if (robot == RobotType.SIM) { // Invalid robot selected
                invalidRobotAlert.set(true);
                return RobotType.REAL;
            } else {
                return robot;
            }
        } else {
            return robot;
        }
    }

    public static Mode getMode() {
        switch (getRobot()) {
            case REAL:
                return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

            case SIM:
                return Mode.SIM;
            default:
                return Mode.REAL;
        }
    }

    public static enum RobotType {
        REAL, SIM
    }

    public static enum Mode {
        REAL, REPLAY, SIM
    }
}
