/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {
	//creates joysticks (JoystickDrive is left, JoystickDrive2 is right)
	public Joystick JoystickDrive;
	public Joystick JoystickDrive2;
	public XboxController Xboxtempest;
	public JoystickButton XboxAbutton;
	public JoystickButton XboxXButton;

	public OI() {
		//defines the joysticks as joysticks and assigns left and right
		JoystickDrive = new Joystick(RobotMap.Controller.leftcontrol);
		JoystickDrive2 = new Joystick(RobotMap.Controller.rightcontrol);
		Xboxtempest = new XboxController(RobotMap.Controller.Xboxcontrol);
		XboxAbutton = new JoystickButton(Xboxtempest, RobotMap.Buttons.XboxAbutton);
		XboxAbutton.whileHeld(Robot.drive.Ebrake());
		XboxXButton = new JoystickButton(Xboxtempest, RobotMap.Buttons.XboxXbutton);
		XboxXButton.whileHeld(Robot.drive.DriveStraight());
	}
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
}
