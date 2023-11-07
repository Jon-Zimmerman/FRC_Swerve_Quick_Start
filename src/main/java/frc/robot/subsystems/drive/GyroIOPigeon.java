package frc.robot.subsystems.drive;
import com.ctre.phoenix.sensors.Pigeon2;
import frc.robot.ConstantsSwerve;

public class GyroIOPigeon implements GyroIO{
    public Pigeon2 gyro;
    
    public GyroIOPigeon () {
        gyro = new Pigeon2(ConstantsSwerve.pigeonID,ConstantsSwerve.swerveCANBusDeviceName);
        calibrateGyro();
        zeroGyro();
    }

    public void updateInputs(GyroIOInputs inputs) {
        
        inputs.yawDegrees = (ConstantsSwerve.invertGyro) ? gyro.getYaw()*-1.0 : gyro.getYaw();
        inputs.rollDegrees = gyro.getRoll();
        inputs.pitchDegrees = gyro.getPitch();
        
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void calibrateGyro() {
        gyro.configFactoryDefault();
    }
}

