// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.Drivebase;
import frc.team5431.titan.core.joysticks.CommandXboxController;

public class RobotContainer {
    private final Systems systems = new Systems();
    public final Drivebase drivebase = systems.getDrivebase();

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final AutonLoader autonLoader = new AutonLoader(drivebase);

    public RobotContainer() {

        driver.setDeadzone(0.15);

        drivebase.setDefaultCommand(new DefaultDriveCommand(
                systems,
                () -> modifyAxis(-driver.getLeftY()) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
                () -> modifyAxis(-driver.getLeftX()) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
                () -> modifyAxis(-driver.getRightX()) * Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

        // systems.getArm().setDefaultCommand(systems.getArm().defaultCommand(
        // () -> modifyAxis(driver.getRightTriggerAxis() - driver.getLeftTriggerAxis()),
        // () -> modifyAxis(driver.getRightY())
        // // () -> {
        // // double power = 0.0;
        // // if (driver.rightBumper().getAsBoolean())
        // // power += 1;
        // // if (driver.leftBumper().getAsBoolean())
        // // power -= 1;
        // // return power;
        // // }
        // ));

        // CommandScheduler.getInstance().removeDefaultCommand(systems.getArm());

        configureBindings();

        Robot.periodics.add(Pair.of(() -> {
            SmartDashboard.putNumber("pressure", systems.getCompressor().getPressure());
            SmartDashboard.putBoolean("pressure switch val", systems.getCompressor().getPressureSwitchValue());
        }, 0.3));

        List<WPI_TalonFX> falcons = systems.getDrivebase().getMotors();

        String[] falconNames = new String[] {
                "FLSteer",
                "FLDrive",
                "FRSteer",
                "FRDrive",
                "BLSteer",
                "BLDrive",
                "BRSteer",
                "BRDrive"
        };

        Robot.periodics.add(Pair.of(() -> {
            for (int i = 0; i < falcons.size(); i++) {
                SmartDashboard.putNumber(falconNames[i] + " Temp", falcons.get(i).getTemperature());
            }
        }, 0.1));
    }

    private void configureBindings() {
        // Y button zeros the gyroscope
        driver.y().onTrue(runOnce(drivebase::zeroGyroscope));

        // D-Pad cardinal directions
        driver.povUp().whileTrue(run(
                () -> drivebase.drive(new ChassisSpeeds(Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0, 0)), drivebase));
        driver.povDown().whileTrue(run(
                () -> drivebase.drive(new ChassisSpeeds(-Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0, 0)), drivebase));
        driver.povLeft().whileTrue(run(
                () -> drivebase.drive(new ChassisSpeeds(0, -Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0)), drivebase));
        driver.povRight().whileTrue(run(
                () -> drivebase.drive(new ChassisSpeeds(0, Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0)), drivebase));

        operator.a().onTrue(runOnce(() -> systems.getManipulator().open()));
        operator.b().onTrue(runOnce(() -> systems.getManipulator().close()));
        operator.x().onTrue(runOnce(() -> systems.getDblSol2().toggle()));
        operator.y().onTrue(runOnce(() -> systems.getSglSol1().toggle()));

        operator.leftBumper().onTrue(runOnce(() -> systems.getArm().incrOut(-10)));
        operator.rightBumper().onTrue(runOnce(() -> systems.getArm().incrOut(10)));
        operator.back().onTrue(runOnce(() -> systems.getArm().incrIn(-10))); // elbow runs opposite dir
        operator.start().onTrue(runOnce(() -> systems.getArm().incrIn(10)));
        operator.povLeft().onTrue(runOnce(() -> systems.getArm().incrWrist(-20)));
        operator.povRight().onTrue(runOnce(() -> systems.getArm().incrWrist(20)));
    }

    public Command getAutonomousCommand() {
        // jolly good
        return autonLoader.procureAuton();
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.075);

        value = value * value * value;

        // Square the axis
        // value = Math.copySign(value * value, value);

        return value;
    }

    public void teleopPeriodic() {
        // systems.getIntakeLeft().set(modifyAxis(driver.getLeftY()));
        // SmartDashboard.putNumber("inleft", systems.getIntakeLeft().get());
    }

    public void robotPeriodic() {
    }

    public void teleopInit() {
        systems.getArm().incrOut(0);
        systems.getArm().incrIn(0);
        systems.getArm().incrWrist(0);
    }
}
