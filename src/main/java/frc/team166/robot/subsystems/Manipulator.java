/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.team166.chopshoplib.commands.ActionCommand;
import frc.team166.chopshoplib.commands.CommandChain;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Preferences;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;
import frc.team166.robot.RobotMap.PreferenceStrings;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Manipulator extends PIDSubsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    WPI_TalonSRX deploymentMotor = new WPI_TalonSRX(RobotMap.CAN.DEPLOYMENT_MOTOR);

    WPI_VictorSPX leftRoller = new WPI_VictorSPX(RobotMap.CAN.ROLLER_LEFT);
    WPI_VictorSPX rightRoller = new WPI_VictorSPX(RobotMap.CAN.ROLLER_RIGHT);

    SpeedControllerGroup rollers = new SpeedControllerGroup(leftRoller, rightRoller);

    DoubleSolenoid innerSolenoid = new DoubleSolenoid(RobotMap.Solenoids.MANIPULATOR_SOLENOID_INNER_A,
            RobotMap.Solenoids.MANIPULATOR_SOLENOID_INNER_B);
    DoubleSolenoid outerSolenoid = new DoubleSolenoid(RobotMap.Solenoids.MANIPULATOR_SOLENOID_OUTER_A,
            RobotMap.Solenoids.MANIPULATOR_SOLENOID_OUTER_B);

    AnalogInput irSensor = new AnalogInput(RobotMap.AnalogInputs.IR);

    AnalogPotentiometer potentiometer = new AnalogPotentiometer(RobotMap.AnalogInputs.MANIPULATOR_POTENTIOMETER);

    // inches:
    double ROLLER_RADIUS = 1.4375;
    // ft:
    double DIST_PER_PULSE_INTAKE = (((ROLLER_RADIUS * 2.0 * Math.PI) / 1024.0) / 12.0);

    private static double kP_Manipulator = 0;
    private static double kI_Manipulator = 0;
    private static double kD_Manipulator = 0;
    private static double kF_Manipulator = 0;

    public Manipulator() {
        super("Manipulator (AKA Chadwick)", kP_Manipulator, kI_Manipulator, kD_Manipulator, kF_Manipulator);

        setAbsoluteTolerance(5);

        addChild(rollers);
        addChild(irSensor);

        leftRoller.setInverted(true);
        rightRoller.setInverted(true);

        // Adding Commands To SmartDashboard
        SmartDashboard.putData("Close Outer", CloseOuterManipulator());
        SmartDashboard.putData("Open Outer", OpenOuterManipulator());
        SmartDashboard.putData("Close Inner", CloseInnerManipulator());
        SmartDashboard.putData("Open Inner", OpenInnerManipulator());
        SmartDashboard.putData("Cube Eject", CubeEject());
        SmartDashboard.putData("Cube Pickup", CubePickup());
        // SmartDashboard.putData("cube pickup with lights", CubePickupWithLights(3));
        SmartDashboard.putData("Deploy Manipulator With Joystick", DeployManipulatorWithJoystick());
        SmartDashboard.putData("Re-Enable Potentiometer", enablePID());

        // Preferences Are Wanted In The Constructer So They Can Appear On Live Window
        PreferenceStrings.setDefaultDouble(RobotMap.PreferenceStrings.CUBE_PICKUP_DISTANCE, 0.5);
        PreferenceStrings.setDefaultDouble(RobotMap.PreferenceStrings.DEPLOY_MANIPULATOR_SPEED, 0.5);
        PreferenceStrings.setDefaultDouble(RobotMap.PreferenceStrings.DEPLOY_MANIPULATOR_TIME, 1.5);
        PreferenceStrings.setDefaultDouble(RobotMap.PreferenceStrings.CUBE_EJECT_WAIT_TIME, 5.0);
        PreferenceStrings.setDefaultDouble(RobotMap.PreferenceStrings.MANIPULATOR_HORIZONTAL_INPUT, 2.5);
        PreferenceStrings.setDefaultDouble(RobotMap.PreferenceStrings.MANIPULATOR_MOTOR_INTAKE_SPEED, 0.8);
        PreferenceStrings.setDefaultDouble(RobotMap.PreferenceStrings.MANIPULATOR_MOTOR_DISCHARGE_SPEED, -0.8);
    }

    // METHODS  
    public void reset() {
        rollers.stopMotor();
        deploymentMotor.stopMotor();
    }

    private void openInnerManipulator() {
        innerSolenoid.set(Value.kForward);
    }

    private void closeInnerManipulator() {
        innerSolenoid.set(Value.kReverse);
    }

    private void openOuterManipulator() {
        outerSolenoid.set(Value.kForward);
    }

    private void closeOuterManipulator() {
        outerSolenoid.set(Value.kReverse);
    }

    private double getIRDistance() {
        return irSensor.getVoltage();
    }

    protected double returnPIDInput() {
        return potentiometer.pidGet();
    }

    protected void usePIDOutput(double output) {
        deploymentMotor.set(output);
    }

    /**
     * Sets motors to intake mode
     * <p>
     * Turns motors on to intake a cube
     */
    private void setMotorsToIntake() {
        //change once you find optimal motor speed
        rollers.set(
                Preferences.getInstance().getDouble(RobotMap.PreferenceStrings.MANIPULATOR_MOTOR_INTAKE_SPEED, 0.4));
    }

    /**
     * Sets motors to discharge mode
     * <p>
     * Turns motors on to discharge a cube
     */
    private void setMotorsToDischarge() {
        rollers.set(Preferences.getInstance().getDouble(RobotMap.PreferenceStrings.MANIPULATOR_MOTOR_DISCHARGE_SPEED,
                -0.4));
        // change once you find optimal motor speed
    }

    // COMMANDS
    public void initDefaultCommand() {
    };

    public Command CloseOuterManipulator() {
        return new ActionCommand("Close Outer Manipulator", this, this::closeOuterManipulator);
    }

    public Command OpenOuterManipulator() {
        return new ActionCommand("Open Outer Manipulator", this, this::openOuterManipulator);
    }

    public Command OpenInnerManipulator() {
        return new ActionCommand("Open Inner Manipulator", this, this::openInnerManipulator);
    }

    public Command CloseInnerManipulator() {
        return new ActionCommand("Close Inner Manipulator", this, this::closeInnerManipulator);
    }

    public Command CubeDrop() {
        return new ActionCommand("Drop Cube", this, this::openInnerManipulator);
    }

    public Command CubeEject() {
        return new SubsystemCommand("Eject Cube", this) {

            @Override
            protected void initialize() {
                setTimeout(Preferences.getInstance().getDouble(RobotMap.PreferenceStrings.CUBE_EJECT_WAIT_TIME, 2.0));
                setMotorsToDischarge();
            }

            @Override
            protected void execute() {

            }

            @Override
            protected boolean isFinished() {

                return isTimedOut();
            }

            @Override
            protected void end() {
                rollers.stopMotor();
            }

            @Override
            protected void interrupted() {
                end();
            }
        };
    }

    public Command CubePickup() {
        return new SubsystemCommand("Pick Up Cube", this) {

            @Override
            protected void initialize() {
                openOuterManipulator();
                setMotorsToIntake();
            }

            @Override
            protected boolean isFinished() {
                if (getIRDistance() > Preferences.getInstance()
                        .getDouble(RobotMap.PreferenceStrings.CUBE_PICKUP_DISTANCE, 1.0)) {
                    return true;
                }
                return false;
            }

            @Override
            protected void end() {
                closeOuterManipulator();
                rollers.stopMotor();
            }
        };
    }

    // public Command CubePickupWithLights(int blinkCount) {
    //     return new CommandChain("Cube Pickup with Lights").then(CubePickup()).then(Robot.led.blinkGreen(blinkCount));
    // }

    public Command DeployManipulator() {
        return new SubsystemCommand("Deploy Manipulator", this) {

            @Override
            protected void initialize() {
                setSetpoint(Preferences.getInstance().getDouble(RobotMap.PreferenceStrings.MANIPULATOR_HORIZONTAL_INPUT,
                        2.5));
            }

            @Override
            protected boolean isFinished() {

                return onTarget();
            }
        };
    }

    public Command DeployManipulatorWithJoystick() {
        return new SubsystemCommand("Deploy Manipulator With Joystick", this) {
            @Override
            protected void initialize() {
                disable();
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void execute() {
                deploymentMotor.set(0);
            }
        };
    }

    public Command enablePID() {
        return new ActionCommand("Enable PID", this, this::enable);
    }

}