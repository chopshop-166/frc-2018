/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LED extends Subsystem {

    public void initDefaultCommand() {
    };

    //these will be changed from DigitalOutputs to something else when we get real hardware...
    DigitalOutput red = new DigitalOutput(0);
    DigitalOutput green = new DigitalOutput(1);
    DigitalOutput blue = new DigitalOutput(2);

    public LED() {

    }

    // METHODS
    private void allOff() {
        red.set(false);
        green.set(false);
        blue.set(false);
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
            red.set(false);
            if (turnOn) {
                blue.set(true);
            } else {
                blue.set(false);
            }
        } else {
            blue.set(false);
            if (turnOn) {
                red.set(true);
            } else {
                red.set(false);
            }
        }
    }

    // COMMANDS
    public Command blinkGreen(int numberOfBlinks) {
        return new SubsystemCommand(this) {
            double lastUpdateTime = System.currentTimeMillis();
            boolean isOn = true;
            double count = 0;

            @Override
            protected void initialize() {
                count = 0;
                green.set(true);
            }

            @Override
            protected void execute() {
                if (System.currentTimeMillis() >= lastUpdateTime + 250) {
                    lastUpdateTime = System.currentTimeMillis();
                    if (isOn) {
                        green.set(false);
                        isOn = false;
                        count++;
                    } else {
                        green.set(true);
                        isOn = true;

                    }
                }
            }

            @Override
            protected boolean isFinished() {
                if (count >= numberOfBlinks) {
                    return true;
                } else {
                    return false;
                }
            }

            @Override
            protected void end() {
                green.set(false);
            }
        };
    }

    public Command blinkTeamColor() {
        return new SubsystemCommand(this) {
            double lastUpdateTime = System.currentTimeMillis();
            boolean isOn = true;

            @Override
            protected void initialize() {
                setTeamColor(true);
            }

            @Override
            protected void execute() {
                if (System.currentTimeMillis() >= lastUpdateTime + 750) {
                    lastUpdateTime = System.currentTimeMillis();
                    if (isOn) {
                        setTeamColor(false);
                        isOn = false;
                    } else {
                        setTeamColor(true);
                        isOn = true;
                    }
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

    public Command blueOn() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                blue.set(true);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                blue.set(false);
            }
        };
    }

    public Command cyanOn() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                red.set(false);
                blue.set(true);
                green.set(true);
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

    public Command greenOn() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                green.set(true);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                green.set(false);
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

    public Command redOn() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                red.set(true);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                red.set(false);
            }
        };
    }

}
