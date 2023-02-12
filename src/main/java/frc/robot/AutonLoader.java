package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Drivebase;

// init sendablechooser in robotcontainer's constructor ✅
// 	getfullauto retreives path groups
// 	(add them to the chooser)

// def getAutonomousCommand()
// {
// 	return chooser.getSelected()
// }

// TODO: Custom widget!!! That complains if you don't choose...
public class AutonLoader {
    private Drivebase drivebase;

    /**
     * Gets initialized in {@link AutonLoader#AutonLoader(Drivebase)}.
     * 
     * IF YOU HAVE A NULLREFERENCE OR UNINITIALIZED BUG, IT'S POSSIBLY FROM HERE.
     */
    private SwerveAutoBuilder autoBuilder = null;

    
    private final SendableChooser<Command> chooser = new SendableChooser<>();
    private final GenericEntry shouldBalance = Shuffleboard.getTab("My Tab")
      .add("My Number", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1))
      .getEntry();

    public AutonLoader(Drivebase drivebase) {
        this.drivebase = drivebase;

        // This is just an example event map. It would be better to have a constant,
        // global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        // Create the AutoBuilder. This only needs to be created once when robot code
        // starts, not every time you want to create an auto command. A good place to
        // put this is in RobotContainer along with your subsystems.
        autoBuilder = new SwerveAutoBuilder(
                drivebase::getPosition, // Pose2d supplier
                drivebase::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                drivebase.m_kinematics, // SwerveDriveKinematics
                Constants.TRANSLATION_PID, // PID constants to correct for translation error (used to create the X
                                           // and Y PID controllers)
                Constants.ROTATION_PID, // PID constants to correct for rotation error (used to create the
                                        // rotation controller)
                (states) -> drivebase.driveRaw(drivebase.m_kinematics.toChassisSpeeds(states)), // Module states
                                                                                                // consumer used to
                                                                                                // output to the drive
                                                                                                // subsystem
                eventMap,
                true, // Should the path be automatically mirrored depending on alliance color.
                      // Optional, defaults to true
                drivebase // The drive subsystem. Used to properly set the requirements of path following
                          // commands
        );


        Command finalCommand = null;

        // for (Filename iterable_element : iterable) {
            
        // }

        shouldBalance.getBoolean(false);
    }

    public Command getFullAuto(String pathName) {
        var pathGroup = PathPlanner.loadPathGroup(pathName, Constants.PATH_CONSTRAINTS);

        return autoBuilder.fullAuto(pathGroup);
    }

    public Command procureAuton() {
        return chooser.getSelected();
    }
}