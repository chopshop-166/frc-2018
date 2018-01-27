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
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Manipulator extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	WPI_TalonSRX leftRoller = new WPI_TalonSRX(RobotMap.CAN.rollerLeft);
	WPI_TalonSRX rightRoller = new WPI_TalonSRX(RobotMap.CAN.rollerRight);
	SpeedControllerGroup rollers = new SpeedControllerGroup(leftRoller, rightRoller);

	AnalogInput irSensor = new AnalogInput(RobotMap.AnalogInputs.ir);

	Encoder leftIntakeEncoder = new Encoder(RobotMap.Encoders.leftRollerA, RobotMap.Encoders.leftRollerB);
	Encoder rightIntakeEncoder = new Encoder(RobotMap.Encoders.rightRollerA, RobotMap.Encoders.rightRollerB);

	double rollerRadius = 1.4375; //inches
	double distPerPulseIntake = (((rollerRadius * 2.0 * Math.PI) / 1024.0) / 12.0); //feet
	double motorSpeed;

	public Manipulator() {
		addChild(rollers);
		addChild(irSensor);
		addChild(leftIntakeEncoder);
		addChild(rightIntakeEncoder);

		leftIntakeEncoder.setReverseDirection(true);
		leftRoller.setInverted(true);

		leftIntakeEncoder.setDistancePerPulse(distPerPulseIntake);
		rightIntakeEncoder.setDistancePerPulse(distPerPulseIntake);
	}

	public void resetEncoders() {
		leftIntakeEncoder.reset();
		rightIntakeEncoder.reset();
	}

	public double irDistance() {
		return irSensor.getVoltage(); //Manipulate this data pending experimentation
	}

	public double avgEncoderRate() { //ft/s
		double leftRate = leftIntakeEncoder.getRate();
		double rightRate = rightIntakeEncoder.getRate();
		return (leftRate + rightRate) / 2.0;
	}

	public double avgEncoderDistance() { //ft
		double leftDist = leftIntakeEncoder.getDistance();
		double rightDist = rightIntakeEncoder.getDistance();
		return (leftDist + rightDist) / 2;
	}

	public void setMotorsToIntake() {
		rollers.set(0.8); //change once you find optimal motor speed
	}

	public void setMotorsToDischarge() {
		rollers.set(-0.8);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new SubsystemCommand(this) {

			@Override
			protected boolean isFinished() {
				return false;
			}

		});
	}

	public Command CubePickup() {
		return new SubsystemCommand(this) {

			@Override
			protected void initialize() {
				resetEncoders();
				rollers.set(0);
				motorSpeed = 0;
			}

			@Override
			protected void execute() {
				//Figure out how to make the motors go 542.865 rpm or 6.81 ft/s without making them spool up
				if (avgEncoderRate() < 6.81)
					motorSpeed += 0.02;

				rollers.set(motorSpeed);
			}

			@Override
			protected boolean isFinished() {
				return irSensor.getValue() == 2.0; //Change this once Mech has a prototype
			}

			@Override
			protected void end() {
				rollers.stopMotor();
			}
		};
	}

	public Command CubeEject() {
		return new SubsystemCommand(this) {

			@Override
			protected void initialize() {
				resetEncoders();
			}

			@Override
			protected void execute() {
				rollers.set(-0.8); //Change this once you figure out the optimal speed of the motors
			}

			@Override
			protected boolean isFinished() {
				return avgEncoderDistance() > 3.0;
			}

			@Override
			protected void end() {
				rollers.stopMotor();
			}
		};
	}
}