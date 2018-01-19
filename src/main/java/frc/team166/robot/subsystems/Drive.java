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
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.team166.robot.PIDOutputVariable;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;
import frc.team166.robot.commands.SubsystemCommand;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drive extends Subsystem {
	AnalogGyro tempestGyro = new AnalogGyro(RobotMap.AnalogInputs.tempestgyro);
	WPI_TalonSRX m_rearleft = new WPI_TalonSRX(RobotMap.CAN.backleft);
	WPI_TalonSRX m_frontleft = new WPI_TalonSRX(RobotMap.CAN.frontleft);
	SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontleft, m_rearleft);

	WPI_TalonSRX m_rearright = new WPI_TalonSRX(RobotMap.CAN.backright);
	WPI_TalonSRX m_frontright = new WPI_TalonSRX(RobotMap.CAN.frontright);
	SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontright, m_rearright);

	DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

	PIDOutputVariable gyroOutput = new PIDOutputVariable();
	final static double kP = 1.0 / 90.0;
	final static double kI = 0.0003;
	final static double kD = 0;
	final static double kF = 0;
	PIDController drivePidController = new PIDController(kP, kI, kD, kF, tempestGyro, gyroOutput);

	double angle;

	public Drive() {
		addChild(tempestGyro);
		addChild(m_drive);
		addChild(drivePidController);
		drivePidController.enable();
	}

	public void initDefaultCommand() {
		setDefaultCommand(new SubsystemCommand(this) {

			@Override
			protected boolean isFinished() {
				return false;
			}

			@Override
			protected void execute() {
				angle = tempestGyro.getAngle();
				drivePidController.setSetpoint(0);
				m_drive.arcadeDrive(-Robot.m_oi.JoystickDrive.getY(),
						Robot.m_oi.JoystickDrive.getX() + gyroOutput.get());
			}
		});
	}
}
