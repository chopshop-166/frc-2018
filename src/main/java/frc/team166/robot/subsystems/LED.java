/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot.subsystems;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.chopshoplib.commands.SubsystemCommand;

public class LED extends Subsystem {
    //these will be changed from Victors to something else when we get real hardware...
    Victor blue = new Victor(1);
    Victor green = new Victor(2);
    Victor red = new Victor(3);

    public LED() {
        SmartDashboard.putData("green", greenOn());
        SmartDashboard.putData("red", redOn());
        SmartDashboard.putData("blue", blueOn());
        SmartDashboard.putData("cyan", cyanOn());
        SmartDashboard.putData("color cycle", colorCycle());
        SmartDashboard.putData("light team color", lightTeamColor());
        SmartDashboard.putData("flash team color", blinkTeamColor());
        SmartDashboard.putData("pulse team color", pulseTeamColor());
    }

    private boolean isBlueTeam() {
        Alliance team = DriverStation.getInstance().getAlliance();
        if (team == DriverStation.Alliance.Blue) {
            return true;
        } else {
            return false;
        }
    }

    private void setTeamColor(boolean turnOn) {
        if (isBlueTeam()) {
            red.set(0);
            if (turnOn) {
                blue.set(1);
            } else {
                blue.set(0);
            }
        } else {
            blue.set(0);
            if (turnOn) {
                red.set(1);
            } else {
                red.set(0);
            }
        }
    }

    private boolean pulseColor(Victor color, double step, double delay, boolean goUp) {
        if (color.get() >= 1) {
            goUp = false;
        }
        if (color.get() <= 0) {
            goUp = true;
        }
        Timer.delay(delay);
        if (goUp) {
            color.set(color.get() + step);
        } else {
            color.set(color.get() - step);
        }
        return goUp;
    }

    private void allOff() {
        red.set(0);
        green.set(0);
        blue.set(0);
    }

    public void initDefaultCommand() {

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public Command greenOn() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                green.set(1);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                green.set(0);
            }
        };
    }

    public Command redOn() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                red.set(1);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                red.set(0);
            }
        };
    }

    public Command blueOn() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                blue.set(1);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                blue.set(0);
            }
        };
    }

    public Command cyanOn() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                red.set(0);
                blue.set(1);
                green.set(1);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                allOff();
            }
        };
    }

    public Command colorCycle() {
        return new SubsystemCommand(this) {
            boolean goUpGreen = true;
            boolean goUpBlue = true;
            boolean goUpRed = true;
            double step = 0.05;
            double delay = 0.01;

            @Override
            protected void initialize() {
                green.set(0);
                blue.set(1);
                red.set(0.5);
            }

            @Override
            protected void execute() {
                goUpGreen = pulseColor(green, step, delay, goUpGreen);
                goUpBlue = pulseColor(blue, step, delay * 2, goUpBlue);
                goUpRed = pulseColor(red, step, delay * 3, goUpRed);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                allOff();
            }
        };
    }

    public Command lightTeamColor() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                setTeamColor(true);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                setTeamColor(false);
            }
        };
    }

    public Command blinkTeamColor() {
        return new SubsystemCommand(this) {
            double delay = 0.5;

            @Override
            protected void initialize() {

            }

            @Override
            protected void execute() {
                setTeamColor(true);
                Timer.delay(delay);
                setTeamColor(false);
                Timer.delay(delay);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                setTeamColor(false);
            }
        };
    }

    public Command pulseTeamColor() {
        return new SubsystemCommand(this) {
            boolean goUpBlue = true;
            boolean goUpRed = true;
            double step = 0.05;
            double delay = 0.05;

            @Override
            protected void initialize() {

            }

            @Override
            protected void execute() {
                if (isBlueTeam()) {
                    red.set(0);
                    goUpBlue = pulseColor(blue, step, delay, goUpBlue);
                } else {
                    blue.set(0);
                    goUpRed = pulseColor(red, step, delay, goUpRed);
                }
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                setTeamColor(false);
            }
        };
    }
}
