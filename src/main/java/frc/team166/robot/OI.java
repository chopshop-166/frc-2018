/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot;

import frc.team166.chopshoplib.controls.ButtonJoystick;
import frc.team166.chopshoplib.controls.ButtonXboxController;;

public class OI {
    // Creates joysticks
    public ButtonJoystick leftDriveStick;
    public ButtonJoystick rightDriveStick;
    public ButtonXboxController xBoxTempest;

    public OI() {
        //defines the joysticks as joysticks and assigns left and right
        leftDriveStick = new ButtonJoystick(RobotMap.Controller.leftcontrol);
        rightDriveStick = new ButtonJoystick(RobotMap.Controller.rightcontrol);
        xBoxTempest = new ButtonXboxController(RobotMap.Controller.Xboxcontrol);
        xBoxTempest.getButton(ButtonXboxController.xBoxButton.kBumperLeft.get())
                .whenReleased(Robot.flipper.activateFlipper());
        xBoxTempest.getButton(ButtonXboxController.xBoxButton.kBumperRight.get())
                .whenReleased(Robot.flipper.deactivateFlipper());
        xBoxTempest.getButton(ButtonXboxController.xBoxButton.kA.get()).whenReleased(Robot.flipper.toggleFlipper());

        /* Add controls to Right Joystick */
        rightDriveStick.getButton(1).whenReleased(Robot.flipper.toggleFlipper());
        rightDriveStick.getButton(4).whenReleased(Robot.flipper.activateFlipper());
        rightDriveStick.getButton(5).whenReleased(Robot.flipper.deactivateFlipper());

        leftDriveStick.getButton(1).whenReleased(Robot.flipper.toggleFlipper());
        leftDriveStick.getButton(4).whenReleased(Robot.flipper.activateFlipper());
        leftDriveStick.getButton(5).whenReleased(Robot.flipper.deactivateFlipper());
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
    // button.whenReleased(new ExampleCommand());

    // Start the command when the button is released and let it run the command 
    // until it is finished as determined by it's isFinished method. 
    // button.whenReleased(new ExampleCommand()); 
}
