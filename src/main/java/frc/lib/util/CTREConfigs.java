package frc.lib.util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.ConstantsSwerve;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            ConstantsSwerve.angleEnableCurrentLimit, 
            ConstantsSwerve.angleContinuousCurrentLimit, 
            ConstantsSwerve.anglePeakCurrentLimit, 
            ConstantsSwerve.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = ConstantsSwerve.angleKP;
        swerveAngleFXConfig.slot0.kI = ConstantsSwerve.angleKI;
        swerveAngleFXConfig.slot0.kD = ConstantsSwerve.angleKD;
        swerveAngleFXConfig.slot0.kF = ConstantsSwerve.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            ConstantsSwerve.driveEnableCurrentLimit, 
            ConstantsSwerve.driveContinuousCurrentLimit, 
            ConstantsSwerve.drivePeakCurrentLimit, 
            ConstantsSwerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = ConstantsSwerve.driveKP;
        swerveDriveFXConfig.slot0.kI = ConstantsSwerve.driveKI;
        swerveDriveFXConfig.slot0.kD = ConstantsSwerve.driveKD;
        swerveDriveFXConfig.slot0.kF = ConstantsSwerve.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = ConstantsSwerve.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = ConstantsSwerve.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = ConstantsSwerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}