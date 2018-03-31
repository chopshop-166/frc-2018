/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot;

//import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.chopshoplib.commands.CommandChain;
import frc.team166.robot.subsystems.Drive;
import frc.team166.robot.subsystems.Manipulator;
import frc.team166.robot.subsystems.LED;
import frc.team166.robot.subsystems.Lift;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static final LED led = new LED();
    public static final Drive drive = new Drive();
    public static final Manipulator manipulator = new Manipulator();
    public static final Lift lift = new Lift();
    public static OI m_oi;
    public static final Compressor compressy = new Compressor(1);

    Command m_autonomousCommand;
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        m_oi = new OI();
        m_chooser.addDefault("Default Auto", drive.DriveTime(3, 0.6));
        m_chooser.addObject("Mid Auto", MidAuto());
        m_chooser.addObject("Cross Line And Drop Cube", CrossLineAndDropCube());
        SmartDashboard.putData("Auto mode", m_chooser);
        CameraServer.getInstance().startAutomaticCapture();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
     * the robot is disabled.
     */
    @Override
    public void disabledInit() {
        drive.reset();
        lift.reset();
        manipulator.reset();

    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString code to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons
     * to the switch structure below with additional strings & commands.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_chooser.getSelected();

        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector",
         * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
         * = new MyAutoCommand(); break; case "Default Auto": default:
         * autonomousCommand = new ExampleCommand(); break; }
         */

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.start();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    public Command CrossLineAndDropCube() {
        return new CommandChain("Cross Line And Drop Cube").then(lift.LowerLiftForTime())
                .then(drive.DriveTime(2.5, 0.6), (lift.RaiseLiftALittle())).then(manipulator.CubeEject());
        //return new CommandChain("Cross Line And Drop Cube").then(lift.LowerLiftForTime());

    }

    public Command MidAuto() {
        String gameData;

        gameData = DriverStation.getInstance().getGameSpecificMessage();

        double degrees = 0.0;
        if (gameData.length() > 0) {
            if (gameData.charAt(0) == 'R') {
                //"R" is for RIGHT NOT RED
                degrees = 90.00;
                //turning right                
                System.out.println("Right");

            } else {
                degrees = -90.00;
                //turning left               
                System.out.println("Left");
            }
        }
        Command cmdMidAuto = new CommandChain("Mid Auto").then(drive.DriveTime(.75, .6), (lift.LowerLiftForTime()))
                .then(drive.TurnByDegrees(degrees)).then(drive.DriveTime(.5, .6)).then(drive.TurnByDegrees(-degrees))
                .then(drive.DriveTime(.3, .6), lift.RaiseLiftALittle()).then(manipulator.CubeDrop());
        return cmdMidAuto;

    }

    private Command ScaleAutoSide(char side, double degrees) {
        String gameData;
        Command cmdAuto = new CommandChain("Autonomous");
        //The line above is to make VS Code happy and to take care of syntax errors
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            if (gameData.charAt(1) == side) {
                //we gon do the scale. Put in corner
                cmdAuto = new CommandChain("Scale Auto").then(drive.DriveTime(1.3, .6), lift.LowerLiftForTime())
                        .then(lift.GoToTopLift()).then(drive.TurnByDegrees(degrees)).then(manipulator.CubeDrop());
            } else if (gameData.charAt(0) == side) {
                //move forward and eject on switch. Align right behind switch
                cmdAuto = new CommandChain("Switch Auto").then(drive.DriveTime(.3, .6), lift.LowerLiftForTime())
                        .then(drive.TurnByDegrees(degrees)).then(lift.RaiseLiftALittle()).then(manipulator.CubeDrop());
            }

            else { //just cross the line
                cmdAuto = new CommandChain("Cross Line Auto").then(drive.DriveTime(.3, .6));

            }
        }
        return cmdAuto;
    }

    public Command ScaleAutoLeftStart() {
        return ScaleAutoSide('L', 90.0);
    }

    public Command ScaleAutoRightStart() {
        return ScaleAutoSide('R', -90.0);
    }

}
