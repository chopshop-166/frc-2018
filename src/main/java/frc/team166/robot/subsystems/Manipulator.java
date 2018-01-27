/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Command;

import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;
import frc.team166.robot.commands.SubsystemCommand;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Manipulator extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	WPI_TalonSRX leftRoller = new WPI_TalonSRX(RobotMap.CAN.rollerLeft);
	WPI_TalonSRX rightRoller = new WPI_TalonSRX(RobotMap.CAN.rollerRight);

	AnalogInput irSensor = new AnalogInput(RobotMap.AnalogInputs.ir);

	Encoder leftIntakeEncoder = new Encoder(RobotMap.Encoders.leftRollerA, RobotMap.Encoders.leftRollerB);
	Encoder rightIntakeEncoder = new Encoder(RobotMap.Encoders.rightRollerA, RobotMap.Encoders.rightRollerB);

	double rollerRadius = 1.4375; //inches
	double distPerPulseIntake = (((rollerRadius * 2.0 * Math.PI) / 1024.0) / 12.0); //feet
	//	double leftRollerRate;
	//	double rightRollerRate;

	public Manipulator() {
		addChild(leftRoller);
		addChild(rightRoller);
		addChild(irSensor);
		addChild(leftIntakeEncoder);
		addChild(rightIntakeEncoder);

		leftIntakeEncoder.setReverseDirection(true);
		leftRoller.setInverted(true);

		leftIntakeEncoder.setDistancePerPulse(distPerPulseIntake);
		rightIntakeEncoder.setDistancePerPulse(distPerPulseIntake);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new SubsystemCommand(this) {

			@Override
			protected void initialize() {
				leftIntakeEncoder.reset();
				rightIntakeEncoder.reset();
			}

			@Override
			protected void execute() {
				//Figure out how to make the motors go 542.865 rpm
			}

			@Override
			protected boolean isFinished() {
				return irSensor.getValue() == 2.0; //Change this once Mech has a prototype
			}

			@Override
			protected void end() {
				leftRoller.stopMotor();
				rightRoller.stopMotor();
			}
		});
	}

	public Command CubePickup() {
		return new SubsystemCommand(this) {

			@Override
			protected void initialize() {

			}

			@Override
			protected void execute() {
				leftRoller.set(0.8);
				rightRoller.set(-0.8); //inverted because it's on the opposite side
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
