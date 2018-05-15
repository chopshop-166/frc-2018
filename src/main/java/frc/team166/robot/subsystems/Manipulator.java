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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;
import frc.team166.robot.RobotMap.PreferenceStrings;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Manipulator extends PIDSubsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    WPI_VictorSPX deploymentMotor = new WPI_VictorSPX(RobotMap.CAN.DEPLOYMENT_MOTOR);

    WPI_VictorSPX leftRoller = new WPI_VictorSPX(RobotMap.CAN.ROLLER_LEFT);
    WPI_TalonSRX rightRoller = new WPI_TalonSRX(RobotMap.CAN.ROLLER_RIGHT);

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
        super("Manipulator", kP_Manipulator, kI_Manipulator, kD_Manipulator, kF_Manipulator);

        setAbsoluteTolerance(5);

        addChild(rollers);
        addChild("IR", irSensor);
        addChild("Potentiometer", potentiometer);
        addChild("Deploy Motor", deploymentMotor);
        addChild("Rollers", rollers);
        addChild("Inner", innerSolenoid);
        addChild("Outer", outerSolenoid);

        leftRoller.setInverted(false);
        rightRoller.setInverted(true);
        deploymentMotor.setInverted(true);

        // Adding Commands To SmartDashboard
        // SmartDashboard.putData("Close Outer", CloseOuterManipulator());
        // SmartDashboard.putData("Open Outer", OpenOuterManipulator());
        // SmartDashboard.putData("Close Inner", CloseInnerManipulator());
        // SmartDashboard.putData("Open Inner", OpenInnerManipulator());
        // SmartDashboard.putData("Cube Eject", CubeEject());
        // SmartDashboard.putData("Cube Pickup", CubePickup());
        // SmartDashboard.putData("cube pickup with lights", CubePickupWithLights(3));
        // SmartDashboard.putData("Deploy Manipulator With Joystick",
        // DeployManipulatorWithJoystick());
        // SmartDashboard.putData("Re-Enable Potentiometer", enablePID());

        // Preferences Are Wanted In The Constructer So They Can Appear On Live Window
        PreferenceStrings.setDefaultDouble(RobotMap.PreferenceStrings.CUBE_PICKUP_DISTANCE, 0.5);
        PreferenceStrings.setDefaultDouble(RobotMap.PreferenceStrings.DEPLOY_MANIPULATOR_SPEED, 0.5);
        PreferenceStrings.setDefaultDouble(RobotMap.PreferenceStrings.DEPLOY_MANIPULATOR_TIME, 1.5);
        PreferenceStrings.setDefaultDouble(RobotMap.PreferenceStrings.CUBE_EJECT_WAIT_TIME, 5.0);
        PreferenceStrings.setDefaultDouble(RobotMap.PreferenceStrings.MANIPULATOR_HORIZONTAL_INPUT, 2.5);
        PreferenceStrings.setDefaultDouble(RobotMap.PreferenceStrings.MANIPULATOR_MOTOR_INTAKE_SPEED, -0.8);
        PreferenceStrings.setDefaultDouble(RobotMap.PreferenceStrings.MANIPULATOR_MOTOR_DISCHARGE_SPEED, 0.8);
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
        // change once you find optimal motor speed
        rollers.set(
                Preferences.getInstance().getDouble(RobotMap.PreferenceStrings.MANIPULATOR_MOTOR_INTAKE_SPEED, -0.6));
    }

    /**
     * Sets motors to discharge mode
     * <p>
     * Turns motors on to discharge a cube
     */
    private void setMotorsToDischarge() {
        rollers.set(
                Preferences.getInstance().getDouble(RobotMap.PreferenceStrings.MANIPULATOR_MOTOR_DISCHARGE_SPEED, 0.6));
        // change once you find optimal motor speed
    }

    // COMMANDS
    public void initDefaultCommand() {
        setDefaultCommand(DefaultCommand());
    };

    public Command DefaultCommand() {
        return new ActionCommand("Manipulator Default Command", this, () -> {
            DeployManipulatorWithJoystick().start();
        });
    }

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

    public Command ManipulatorDischarge() {
        return new ActionCommand("DisCharge Manipulator", this, this::setMotorsToDischarge);
    }

    public Command ManipulatorIntake() {
        return new ActionCommand("Intake Manipulator", this, this::setMotorsToIntake);
    }

    public Command CubeDrop() {
        return new ActionCommand("Drop Cube", this, this::openInnerManipulator);
    }

    public Command CubeClamp() {
        return new ActionCommand("Cube Clamp", this, this::closeInnerManipulator);
    }

    public Command ManipulatorIntakeHeld() {
        return new SubsystemCommand("Intake", this) {

            @Override
            protected void initialize() {
                setMotorsToIntake();
            }

            @Override
            protected boolean isFinished() {
                return false;

            }

            @Override
            protected void end() {
                rollers.set(0);
            }

            @Override
            protected void interrupted() {
                end();
            }

        };
    }

    public Command ManipulatorDischargeHeld() {
        return new SubsystemCommand("Discharge", this) {

            @Override
            protected void initialize() {
                setMotorsToDischarge();
            }

            @Override
            protected boolean isFinished() {
                return false;

            }

            @Override
            protected void end() {
                rollers.set(0);
            }

            @Override
            protected void interrupted() {
                end();
            }

        };
    }

    public Command CubeEject() {
        return new SubsystemCommand("Eject Cube", this) {

            @Override
            protected void initialize() {
                setTimeout(Preferences.getInstance().getDouble(RobotMap.PreferenceStrings.CUBE_EJECT_WAIT_TIME, 1));
                String gameData;
                gameData = DriverStation.getInstance().getGameSpecificMessage();

                if (gameData.length() > 0) {
                    if (gameData.charAt(0) == 'R') {
                        setMotorsToDischarge();
                    }
                }
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
                Rumble().start();
            }
        };
    }

    public Command Rumble() {

        return new SubsystemCommand("Rumble", this) {

            @Override
            protected void initialize() {
                Robot.m_oi.xBoxTempest.setRumble(RumbleType.kLeftRumble, 1);
                Robot.m_oi.xBoxTempest.setRumble(RumbleType.kRightRumble, 1);
                setTimeout(.1);
            }

            @Override
            protected boolean isFinished() {
                return isTimedOut();
            }

            @Override
            protected void end() {
                Robot.m_oi.xBoxTempest.setRumble(RumbleType.kLeftRumble, 0);
                Robot.m_oi.xBoxTempest.setRumble(RumbleType.kRightRumble, 0);
            }

        };
    }

    public Command CubePickupWithLights(int blinkCount) {
        return new CommandChain("Cube Pickup with Lights").then(CubePickup()).then(Robot.led.BlinkGreen(blinkCount));
    }

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
        return new SubsystemCommand("Deploy Manipulator With Joystick") {
            double rotation;

            @Override
            protected void initialize() {
                disable();
            }

            @Override
            protected void execute() {
                rotation = Math.pow(Robot.m_oi.xBoxTempest.getY(Hand.kLeft), 2);

                rotation = rotation
                        * (Robot.m_oi.xBoxTempest.getY(Hand.kLeft) / Math.abs(Robot.m_oi.xBoxTempest.getY(Hand.kLeft)));

                deploymentMotor.set(rotation);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

        };
    }

    public Command enablePID() {
        return new ActionCommand("Enable PID", this, this::enable);
    }

}