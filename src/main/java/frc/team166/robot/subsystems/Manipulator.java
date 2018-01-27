/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.robot.RobotMap;

/**
 * The 'manipulatorSolenoid' causes the manipulator to open when it's set to 'false' and
 * the manipulator closes when the 'manipulatorSolenoid' is set to 'true' 
 */
public class Manipulator extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	WPI_TalonSRX leftRoller = new WPI_TalonSRX(RobotMap.CAN.rollerLeft);
	WPI_TalonSRX rightRoller = new WPI_TalonSRX(RobotMap.CAN.rollerRight);
	SpeedControllerGroup rollers = new SpeedControllerGroup(leftRoller, rightRoller);

	DoubleSolenoid manipulatorSolenoid = new DoubleSolenoid(RobotMap.Solenoids.manipulatorSolenoidA,
			RobotMap.Solenoids.manipulatorSolenoidB);

	AnalogInput irSensor = new AnalogInput(RobotMap.AnalogInputs.ir);

	Encoder leftIntakeEncoder = new Encoder(RobotMap.Encoders.leftRollerA, RobotMap.Encoders.leftRollerB);
	Encoder rightIntakeEncoder = new Encoder(RobotMap.Encoders.rightRollerA, RobotMap.Encoders.rightRollerB);

	double ROLLER_RADIUS = 1.4375; //inches
	double DIST_PER_PULSE_INTAKE = (((ROLLER_RADIUS * 2.0 * Math.PI) / 1024.0) / 12.0); //feet
	double OPTIMAL_MOTOR_RATE = 6.81; //ft/s

	double motorSpeed;

	public Manipulator() {
		addChild(rollers);
		addChild(irSensor);
		addChild(leftIntakeEncoder);
		addChild(rightIntakeEncoder);

		leftIntakeEncoder.setReverseDirection(true);
		leftRoller.setInverted(true);

		leftIntakeEncoder.setDistancePerPulse(DIST_PER_PULSE_INTAKE);
		rightIntakeEncoder.setDistancePerPulse(DIST_PER_PULSE_INTAKE);
	}

	/**
	 * Resets encoders
	 * <p>
	 * Calls the reset function on the left and right intake encoders
	 */
	public void resetEncoders() {
		leftIntakeEncoder.reset();
		rightIntakeEncoder.reset();
	}

	/**
	 * Gets IR voltage
	 * <p>
	 * Returns the voltage value output from the IR sensor
	 * 
	 * @return The voltage from the IR sensor
	 */
	public double getIRDistance() {
		return irSensor.getVoltage(); //Manipulate this data pending experimentation
	}

	public void openManipulator() {
		manipulatorSolenoid.set(Value.kForward);
		;
	}

	public void closeManipulator() {
		manipulatorSolenoid.set(Value.kReverse);
	}

	public double avgEncoderRate() { //ft/s
		double leftRate = leftIntakeEncoder.getRate();
		double rightRate = rightIntakeEncoder.getRate();
		return (leftRate + rightRate) / 2.0;
	}

	/**
	 * Gets average encoder distance
	 * <p>
	 * Returns the average distance of the left and right encoders
	 * 
	 * @return The average distance of the two encoders
	 */
	public double getAvgEncoderDistance() { //ft
		double leftDist = leftIntakeEncoder.getDistance();
		double rightDist = rightIntakeEncoder.getDistance();
		return (leftDist + rightDist) / 2.0;
	}

	/**
	 * Sets motors to intake mode
	 * <p>
	 * Sets the motors to 4/5 power inward to take in a cube on the field
	 */
	public void setMotorsToIntake() {
		rollers.set(0.8); //change once you find optimal motor speed
	}

	/**
	 * Sets motors to discharge mode
	 * <p>
	 * Sets the motors to 4/5 power outward to eject a stored cube
	 */
	public void setMotorsToDischarge() {
		rollers.set(-0.8); //change once you find optimal motor speed
	}

	//COMMANDS
	public void initDefaultCommand() {
		setDefaultCommand(new SubsystemCommand(this) {

			@Override
			protected boolean isFinished() {
				return false;
			}

		});
	}

	public Command DropCube() {
		return new SubsystemCommand(this) {

			@Override
			protected void initialize() {
				openManipulator();
			}

			@Override
			protected void execute() {

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

	public Command CloseManipulator() {
		return new SubsystemCommand(this) {

			@Override
			protected void initialize() {
				closeManipulator();
			}

			@Override
			protected void execute() {

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
				if (getAvgEncoderRate() < OPTIMAL_MOTOR_RATE)
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
				return getAvgEncoderDistance() > 3.0;
			}

			@Override
			protected void end() {
				rollers.stopMotor();
			}
		};
	}
}