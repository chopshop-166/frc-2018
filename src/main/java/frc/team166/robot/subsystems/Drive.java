/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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

    //this makes children that control the tempestGyro, drive motors, and PIDController loop. 
    public Drive() {
        addChild(m_drive);
    }

    //the default command for this code is supposed to rotate the robot so that it's gyro value is 0
    public void initDefaultCommand() {
        setDefaultCommand(xBoxArcade());
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

    public Command xBoxArcade() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(
                        Robot.m_oi.xBoxTempest.getTriggerAxis(Hand.kRight)
                                - Robot.m_oi.xBoxTempest.getTriggerAxis(Hand.kLeft),
                        Robot.m_oi.xBoxTempest.getX(Hand.kLeft));
            }

            @Override
            protected boolean isFinished() {
                return false;
            }
        };
    }

    public Command xBoxDriveTank() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
            }

            @Override
            protected void execute() {
                m_drive.tankDrive(-Robot.m_oi.xBoxTempest.getY(Hand.kLeft), -Robot.m_oi.xBoxTempest.getY(Hand.kRight));
            }

            @Override
            protected boolean isFinished() {
                return false;
            }
        };
    }

    public Command xBoxDriveArcadeJoysticks() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(-Robot.m_oi.xBoxTempest.getY(Hand.kLeft), Robot.m_oi.xBoxTempest.getX(Hand.kRight));
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

    public Command joystickTank() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
            }

            @Override
            protected void execute() {
                m_drive.tankDrive(-Robot.m_oi.leftDriveStick.getY(), -Robot.m_oi.rightDriveStick.getY());
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

    public Command joystickArcadeOneStick() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(-Robot.m_oi.leftDriveStick.getY(), Robot.m_oi.leftDriveStick.getX());
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

    public Command joystickArcadeTwoStick() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(-Robot.m_oi.leftDriveStick.getY(), Robot.m_oi.rightDriveStick.getX());
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
}
