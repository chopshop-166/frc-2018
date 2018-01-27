/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;
import frc.team166.chopshoplib.commands.SubsystemCommand;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drive extends Subsystem {

    //defines the gyro
    AnalogGyro tempestGyro = new AnalogGyro(RobotMap.AnalogInputs.tempestgyro);
    //defines the left motors as motors and combines the left motors into one motor
    WPI_TalonSRX m_rearleft = new WPI_TalonSRX(RobotMap.CAN.backleft);
    WPI_TalonSRX m_frontleft = new WPI_TalonSRX(RobotMap.CAN.frontleft);
    SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontleft, m_rearleft);
    //defines the right motors as motors and combines the left motors into one motor
    WPI_TalonSRX m_rearright = new WPI_TalonSRX(RobotMap.CAN.backright);
    WPI_TalonSRX m_frontright = new WPI_TalonSRX(RobotMap.CAN.frontright);
    SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontright, m_rearright);

    /**defines the left and right motors defined above into a differential drive
     * that can be used for arcade and tank drive, amung other things
     */
    DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

    //defines values that will be used in the PIDController (In order of where they will fall in the Controller)
    final static double kP = 1.0 / 180.0;
    final static double kI = 0.0003;
    final static double kD = 1;
    final static double kF = 0;

    //PIDController loop used to find the power of the motors needed to keep the angle of the gyro at 0 
    PIDController drivePidController = new PIDController(kP, kI, kD, kF, tempestGyro, (double value) -> {
        //this assigns the output to the angle (double) defined later in the code)
        angle = value;
    });

    //defines a new double that is going to be used in the line that defines the drive type
    double angle;

    //this makes children that control the tempestGyro, drive motors, and PIDController loop. 
    public Drive() {
        addChild(tempestGyro);
        addChild(m_drive);
        addChild(drivePidController);
        drivePidController.disable();
    }

    //the default command for this code is supposed to rotate the robot so that it's gyro value is 0
    public void initDefaultCommand() {
        setDefaultCommand(new SubsystemCommand(this) {

            /**if there were something else that we wanted the robot, we would use this code to 
            *figure out when to stop the default command.
            */
            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(
                        Robot.m_oi.xBoxTempest.getTriggerAxis(Hand.kRight)
                                - Robot.m_oi.xBoxTempest.getTriggerAxis(Hand.kLeft),
                        Robot.m_oi.xBoxTempest.getX(Hand.kLeft));
            }
        });
    }

    public Command Ebrake() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                m_drive.stopMotor();
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {

            }
        };
    }

    public Command DriveStraight() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                drivePidController.setSetpoint(tempestGyro.getAngle());
                drivePidController.enable();
                drivePidController.reset();
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(Robot.m_oi.xBoxTempest.getTriggerAxis(Hand.kRight)
                        - Robot.m_oi.xBoxTempest.getTriggerAxis(Hand.kLeft), angle);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                drivePidController.disable();
            }
        };
    }
}
