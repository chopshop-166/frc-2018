/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
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
        leftDriveStick.getButton(RobotMap.Buttons.JoystickTrigger).whileHeld(Robot.drive.DriveStraight());
        xBoxTempest = new ButtonXboxController(RobotMap.Controller.Xboxcontrol);
        // xBoxTempest.getButton(RobotMap.Buttons.XboxAbutton).whileHeld(Robot.drive.Ebrake());
        // xBoxTempest.getButton(ButtonXboxController.xBoxButton.kB.get())
        // .whenPressed(Robot.manipulator.CloseOuterManipulator());
        // xBoxTempest.getButton(ButtonXboxController.xBoxButton.kA.get())
        // .whenPressed(Robot.manipulator.OpenOuterManipulator());

        xBoxTempest.getButton(ButtonXboxController.xBoxButton.kA.get())
                .whileHeld(Robot.manipulator.ManipulatorIntakeHeld());
        xBoxTempest.getButton(ButtonXboxController.xBoxButton.kB.get())
                .whileHeld(Robot.manipulator.ManipulatorDischargeHeld());

        xBoxTempest.getButton(ButtonXboxController.xBoxButton.kStickLeft.get())
                .whenPressed(Robot.manipulator.DeployManipulatorWithJoystick());

        // xBoxTempest.getButton(ButtonXboxController.xBoxButton.kY.get())
        //         .whenPressed(Robot.manipulator.CloseInnerManipulator());
        // xBoxTempest.getButton(ButtonXboxController.xBoxButton.kX.get())
        //         .whenPressed(Robot.manipulator.OpenInnerManipulator());

        rightDriveStick.getButton(2).whenPressed(Robot.manipulator.CubeClamp());
        rightDriveStick.getButton(1).whenPressed(Robot.manipulator.CubeDrop());
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
}
