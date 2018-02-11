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
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.chopshoplib.sensors.Lidar;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drive extends Subsystem {

    // declare lidar
    Lidar frontLidar = new Lidar(Port.kOnboard, 0x10);
    //defines the gyro
    AnalogGyro tempestGyro = new AnalogGyro(RobotMap.AnalogInputs.tempestgyro);
    //defines the left motors as motors and combines the left motors into one motor
    WPI_TalonSRX m_rearleft = new WPI_TalonSRX(RobotMap.CAN.BACK_LEFT);
    WPI_TalonSRX m_frontleft = new WPI_TalonSRX(RobotMap.CAN.FRONT_LEFT);
    SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontleft, m_rearleft);
    //defines the right motors as motors and combines the left motors into one motor
    WPI_TalonSRX m_rearright = new WPI_TalonSRX(RobotMap.CAN.BACK_RIGHT);
    WPI_TalonSRX m_frontright = new WPI_TalonSRX(RobotMap.CAN.FRONT_RIGHT);
    SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontright, m_rearright);

    /**defines the left and right motors defined above into a differential drive
     * that can be used for arcade and tank drive, amung other things
     */
    DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

    //defines values that will be used in the PIDController (In order of where they will fall in the Controller)
    final static double kP = 1.0 / 60;
    final static double kI = 0.00003;
    final static double kD = 0;
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
        SmartDashboard.putData(Ebrake());
        SmartDashboard.putData("XBox", xboxArcade());
        SmartDashboard.putData("Turn -45", turnByDegrees(-45));
        SmartDashboard.putData("Turn 45", turnByDegrees(45));

        addChild(tempestGyro);
        addChild(m_drive);
        addChild(drivePidController);
        addChild(frontLidar);

        Preferences.getInstance().getDouble(RobotMap.Preferences.K_P, 1);

        drivePidController.disable();
        drivePidController.setInputRange(0, 360);
        drivePidController.setContinuous();
    }

    //the default command for this code is supposed to rotate the robot so that it's gyro value is 0
    public void initDefaultCommand() {
        setDefaultCommand(joystickArcadeTwoStick());
    }

    public Command xboxArcade() {
        return new SubsystemCommand("XBoxArcade", this) {
            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(-Robot.m_oi.xBoxTempest.getY(Hand.kLeft), Robot.m_oi.xBoxTempest.getX(Hand.kRight));

            }

        };
    }

    public Command Ebrake() {
        return new SubsystemCommand("Ebrake", this) {
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

    public Command joystickArcadeTwoStick() {
        return new SubsystemCommand("joystick Arcade with two sticks", this) {
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
    };

    public Command driveStraight() {
        return new SubsystemCommand("Drive Straight", this) {
            @Override
            protected void initialize() {
                drivePidController.reset();
                drivePidController.setSetpoint(tempestGyro.getAngle());
                drivePidController.enable();
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

    public Command drivetoProximity(double inches) {
        return new SubsystemCommand("Drive Distance", this) {

            //double realDistanceInches = frontLidar.getDistance(true);

            @Override
            protected void initialize() {
                drivePidController.setSetpoint(tempestGyro.getAngle());
                drivePidController.enable();
                drivePidController.reset();
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(Preferences.getInstance().getDouble(RobotMap.Preferences.K_P, 1), angle);
            }

            @Override
            protected boolean isFinished() {
                if (frontLidar.getDistance(true) <= inches) {
                    return true;
                } else {
                    return false;
                }

            }

            @Override
            protected void end() {
                drivePidController.disable();
            }
        };
    }

    public Command turnByDegrees(double degrees) {
        return new SubsystemCommand("Turn " + degrees, this) {
            @Override
            protected void initialize() {
                tempestGyro.reset();
                drivePidController.reset();
                drivePidController.enable();
                drivePidController.setAbsoluteTolerance(3);
                drivePidController.setSetpoint(degrees);
            }

            @Override
            protected void execute() {
                SmartDashboard.putNumber("Drive Angle", angle);
                m_drive.arcadeDrive(0.0, angle);

            }

            @Override
            protected boolean isFinished() {
                return drivePidController.onTarget();
            }

            @Override
            protected void end() {
                drivePidController.disable();
            }
        };
    }

}
