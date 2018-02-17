/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.chopshoplib.commands.ActionCommand;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.robot.RobotMap;

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
    AnalogInput potentiometer = new AnalogInput(RobotMap.AnalogInputs.potentiometer);

    double ROLLER_RADIUS = 1.4375; //inches
    double DIST_PER_PULSE_INTAKE = (((ROLLER_RADIUS * 2.0 * Math.PI) / 1024.0) / 12.0); //feet
    double OPTIMAL_MOTOR_RATE = 6.81; //ft/s

    private static double kP_Manipulator = Preferences.getInstance().getDouble(RobotMap.Preferences.K_P_MANIPULATOR, 1);
    private static double kI_Manipulator = Preferences.getInstance().getDouble(RobotMap.Preferences.K_I_MANIPULATOR, 1);
    private static double kD_Manipulator = Preferences.getInstance().getDouble(RobotMap.Preferences.K_D_MANIPULATOR, 1);
    private static double kF_Manipulator = Preferences.getInstance().getDouble(RobotMap.Preferences.K_F_MANIPULATOR, 1);

    public Manipulator() {
        super("Manipulator (AKA Chadwick)", kP_Manipulator, kI_Manipulator, kD_Manipulator, kF_Manipulator);

        setAbsoluteTolerance(5);

        addChild(rollers);
        addChild("IR Sensor", irSensor);

        SmartDashboard.putData("Close Outer", CloseOuterManipulator());
        SmartDashboard.putData("Open Outer", OpenOuterManipulator());
        SmartDashboard.putData("Close Inner", CloseInnerManipulator());
        SmartDashboard.putData("Open Inner", OpenInnerManipulator());
        SmartDashboard.putData("Cube Pickup", CubePickup());
        SmartDashboard.putData("Cube Eject", CubeEject());

        leftRoller.setInverted(true);
        rightRoller.setInverted(true);

        //Preferences Are Wanted In The Constructer So They Can Appear On Live Window
        Preferences.getInstance().getDouble(RobotMap.Preferences.K_P_MANIPULATOR, 1);
        Preferences.getInstance().getDouble(RobotMap.Preferences.K_I_MANIPULATOR, 1);
        Preferences.getInstance().getDouble(RobotMap.Preferences.K_D_MANIPULATOR, 1);
        Preferences.getInstance().getDouble(RobotMap.Preferences.K_F_MANIPULATOR, 1);
        Preferences.getInstance().getDouble(RobotMap.Preferences.MANIPULATOR_MOTOR_INTAKE_SPEED, 0.8);
        Preferences.getInstance().getDouble(RobotMap.Preferences.MANIPULATOR_MOTOR_DISCHARGE_SPEED, -0.8);
        Preferences.getInstance().getDouble(RobotMap.Preferences.CUBE_PICKUP_DISTANCE, 0.5);
        Preferences.getInstance().getDouble(RobotMap.Preferences.CUBE_EJECT_WAIT_TIME, 5.0);
        Preferences.getInstance().getDouble(RobotMap.Preferences.DEPLOY_MANIPULATOR_TIME, 1.5);
        Preferences.getInstance().getDouble(RobotMap.Preferences.DEPLOY_MANIPULATOR_SPEED, 0.5);
        Preferences.getInstance().getDouble(RobotMap.Preferences.MANIPULATOR_HORIZONTAL_INPUT, 2.5);
    }

    /**
     * Gets IR voltage
     * <p>
     * Returns the voltage value output from the IR sensor
     * 
     * @return The voltage from the IR sensor
     */
    protected double returnPIDInput() {
        return potentiometer.pidGet();
    }

    protected void usePIDOutput(double output) {
        deploymentMotor.set(output);
    }

    private double getIRDistance() {
        return irSensor.getVoltage(); //Manipulate this data pending experimentation
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

    /**
     * Sets motors to intake mode
     * <p>
     * Sets the motors to 4/5 power inward to take in a cube on the field
     */
    private void setMotorsToIntake() {
        //change once you find optimal motor speed
        rollers.set(Preferences.getInstance().getDouble(RobotMap.Preferences.MANIPULATOR_MOTOR_INTAKE_SPEED, 0.4));
    }

    /**
     * Sets motors to discharge mode
     * <p>
     * Sets the motors to 4/5 power outward to eject a stored cube
     */
    private void setMotorsToDischarge() {
        rollers.set(Preferences.getInstance().getDouble(RobotMap.Preferences.MANIPULATOR_MOTOR_DISCHARGE_SPEED, -0.4)); //change once you find optimal motor speed
    }

    //COMMANDS
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

    public Command CubePickup() {
        return new SubsystemCommand("Pick Up Cube", this) {

            @Override
            protected void initialize() {
                openOuterManipulator();
                setMotorsToIntake();
            }

            @Override
            protected boolean isFinished() {
                if (getIRDistance() > Preferences.getInstance().getDouble(RobotMap.Preferences.CUBE_PICKUP_DISTANCE,
                        1.0)) {
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

    public Command CubeEject() {
        return new SubsystemCommand("Eject Cube", this) {

            @Override
            protected void initialize() {
                setTimeout(Preferences.getInstance().getDouble(RobotMap.Preferences.CUBE_EJECT_WAIT_TIME, 2.0));
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

    public Command DeployManipulator() {
        return new SubsystemCommand("Deploy Manipulator", this) {

            @Override
            protected void initialize() {
                setSetpoint(
                        Preferences.getInstance().getDouble(RobotMap.Preferences.MANIPULATOR_HORIZONTAL_INPUT, 2.5));
            }

            @Override
            protected void execute() {

            }

            @Override
            protected boolean isFinished() {

                return onTarget();
            }

            @Override
            protected void interrupted() {
                end();
            }
        };
    }
}