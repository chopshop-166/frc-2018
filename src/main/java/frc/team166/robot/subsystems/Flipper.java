/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.chopshoplib.commands.ActionCommand;
import frc.team166.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Flipper extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    DoubleSolenoid flipper = new DoubleSolenoid(RobotMap.Solenoids.FLIPPER_SOLENOID_A,
            RobotMap.Solenoids.FLIPPER_SOLENOID_B);

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public Command activateFlipper() {
        return new ActionCommand("Activate", () -> {
            flipper.set(DoubleSolenoid.Value.kForward);
            SmartDashboard.putString("TESTING", "Activate");
        });
    }

    public Command deactivateFlipper() {
        return new ActionCommand("Deactivate", () -> {
            flipper.set(DoubleSolenoid.Value.kReverse);
            SmartDashboard.putString("TESTING", "Deactivate");
        });
    }

    public Command toggleFlipper() {
        return new ActionCommand("Toggle Flipper", () -> {
            SmartDashboard.putString("TESTING", "Toggle");
            if (flipper.get() == DoubleSolenoid.Value.kForward) {
                flipper.set(DoubleSolenoid.Value.kReverse);
            } else {
                flipper.set(DoubleSolenoid.Value.kForward);
            }
        });
    }
}
