package frc.robot.autos;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.slider.Slider;
import frc.robot.commands.SliderGoToPosition;
import frc.robot.commands.ElevatorGoToPosition;

public class Example_Auto extends SequentialCommandGroup {
    PathConstraints constraints = new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    List<PathPlannerTrajectory> Path01= PathPlanner.loadPathGroup("Top_Extended_01", constraints);
    List<PathPlannerTrajectory> Path02= PathPlanner.loadPathGroup("Top_Extended_02", constraints);
    List<PathPlannerTrajectory> Path03= PathPlanner.loadPathGroup("Top_Extended_03", constraints);
    List<PathPlannerTrajectory> Path04= PathPlanner.loadPathGroup("Top_Extended_04", constraints);

    final Command Path01Command,Path02Command,Path03Command,Path04Command;

    public Example_Auto(Swerve s_Swerve, Intake intake, Elevator elevator,Slider slider){
        Path01Command = s_Swerve.swerveAutoBuilder.fullAuto (Path01);
        Path02Command = s_Swerve.swerveAutoBuilder.fullAuto (Path02);
        Path03Command = s_Swerve.swerveAutoBuilder.fullAuto (Path03);
        Path04Command = s_Swerve.swerveAutoBuilder.fullAuto (Path04);
                
        addCommands(
            // place first Cube
        new InstantCommand(() -> intake.setIntakeModeCube()),
        s_Swerve.swerveAutoBuilder.resetPose(Path01.get(0)),
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosTop,5.0,elevator).withTimeout(3.0),
        new SliderGoToPosition(Constants.SliderSubsystem.sliderOut,0.5,slider).withTimeout(3.0),
        new StartEndCommand(() ->  intake.intakeOut(),intake::stop,intake).withTimeout(0.5), //make time based
        new SliderGoToPosition(Constants.SliderSubsystem.sliderIn,5.0,slider).withTimeout(3.0),
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosBottom,6.0,elevator).withTimeout(3.0),
        Path01Command,

        // pickup second cube
        new SliderGoToPosition(Constants.SliderSubsystem.sliderOut,0.5,slider).withTimeout(3.0),
        Path02Command, // start the second path so that we move and run intake
        new StartEndCommand(() ->  intake.intakeIn(),intake::stop,intake).withTimeout(2), //make time based

        // turn around
        Path03Command,
        //extend elevator
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosTop,5.0,elevator).withTimeout(3.0),
        new SliderGoToPosition(Constants.SliderSubsystem.sliderOut,0.5,slider).withTimeout(3.0),
        //place second cube on mid
        Path04Command,
        new StartEndCommand(() ->  intake.intakeOut(),intake::stop,intake).withTimeout(0.5), //make time based
        new SliderGoToPosition(Constants.SliderSubsystem.sliderIn,5.0,slider).withTimeout(3.0),
        new ElevatorGoToPosition(Constants.ElevatorSubsystem.elevatorPosBottom,6.0,elevator).withTimeout(3.0)
        // ready for teleop
        );
    }

}