/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.chopshoplib.commands.ActionCommand;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.chopshoplib.outputs.DigitalOutputDutyCycle;
import frc.team166.robot.RobotMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LED extends Subsystem {

    public void initDefaultCommand() {
        setDefaultCommand(BreathTeamColor());
    };

    //these will be changed from DigitalOutputs to something else when we get real hardware...
    DigitalOutputDutyCycle red = new DigitalOutputDutyCycle(RobotMap.DigitalInputs.RED_LED);
    DigitalOutputDutyCycle green = new DigitalOutputDutyCycle(RobotMap.DigitalInputs.GREEN_LED);
    DigitalOutputDutyCycle blue = new DigitalOutputDutyCycle(RobotMap.DigitalInputs.BLUE_LED);

    public LED() {
        registerCommands();
    }

    // METHODS
    private void registerCommands() {
        SmartDashboard.putData("Breath Blue", Breath(blue, 10));
        SmartDashboard.putData("All Off", new ActionCommand("OFF GERALD", this, this::allOff));
    }

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
    public Command BlinkGreen(int numberOfBlinks) {
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

    public Command BlinkTeamColor() {
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

    public Command BlueOn() {
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

    public Command CyanOn() {
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

    public Command GreenOn() {
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

    public Command LightTeamColor() {
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

    public Command RedOn() {
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

    public Command Breath(DigitalOutputDutyCycle color, int frequency) {
        return new SubsystemCommand("fade", this) {
            boolean isDutyCycleIncreasing = true;
            double period;
            final double executePeriod = 20 * 0.001; //Approx how often execute is called
            final double dutyCycleChangePerPeriod = 2.0;
            double changeAmount;

            @Override
            protected void initialize() {
                period = (1.0 / frequency);
                changeAmount = dutyCycleChangePerPeriod / ((period / executePeriod));
                color.enablePWM(0);
                isDutyCycleIncreasing = true;
            }

            @Override
            protected void execute() {
                if (isDutyCycleIncreasing == true) {
                    color.updateDutyCycle(color.getPWMRate() + changeAmount);
                } else {
                    color.updateDutyCycle(color.getPWMRate() - changeAmount);
                }
                if ((color.getPWMRate() >= 1) || (color.getPWMRate() <= 0)) {
                    isDutyCycleIncreasing = !isDutyCycleIncreasing;
                }

            }

            @Override
            protected boolean isFinished() {
                return false;
            }
        };
    }

    private Command BreathTeamColor() {
        return new ActionCommand("Breath Team Color", this, () -> {
            if (isBlueTeam()) {
                red.disablePWM();
                Breath(blue, 2).start();
            } else {
                blue.disablePWM();
                Breath(red, 2).start();
            }
        });
    }

}
