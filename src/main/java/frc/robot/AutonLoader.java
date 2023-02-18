package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.Drivebase;

// init sendablechooser in robotcontainer's constructor ✅
// 	getfullauto retreives path groups
// 	(add them to the chooser)

// def getAutonomousCommand()
// {
// 	return chooser.getSelected()
// }

// TODO: map events to commands

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
	private final GenericEntry shouldBalance = Shuffleboard.getTab("Auton").add("Balance", false)
			.withWidget(BuiltInWidgets.kBooleanBox).getEntry();

	public AutonLoader(Drivebase drivebase) {
		this.drivebase = drivebase;

		HashMap<String, Command> eventMap = new HashMap<>();
		eventMap.put("marker1", new PrintCommand("Passed marker 1"));

		// This can be reused for all autos.
		autoBuilder = new SwerveAutoBuilder(
				drivebase::getPosition,
				drivebase::resetOdometry,
				drivebase.m_kinematics,
				Constants.TRANSLATION_PID,
				Constants.ROTATION_PID,
				(states) -> drivebase.driveRaw(drivebase.m_kinematics.toChassisSpeeds(states)),
				eventMap,
				true,
				drivebase);

		for (String pathName : new String[]{ "path1", "path2" }) {
			chooser.addOption(pathName, getFullAuto(pathName));
			
		}
	}

	public Command getFullAuto(String pathName) {
		var pathGroup = PathPlanner.loadPathGroup(pathName, Constants.PATH_CONSTRAINTS);

		return autoBuilder.fullAuto(pathGroup);
	}

	public Command procureAuton() {
		var mostOfAuto = chooser.getSelected();

		if (shouldBalance.getBoolean(false)) {
			return mostOfAuto.andThen(new AutoAligner(drivebase));
		} else {
			return mostOfAuto;
		}
	}
}