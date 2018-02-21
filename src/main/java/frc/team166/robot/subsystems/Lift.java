/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*Functions 
read encoder 
set motor to raise lift 
set motor to lower lift 
shift into low or high gear 
read top limit switch 
read bottom limit switch 
read other limit switches...? 
 go to specified height

Commands 
go to specified height 
make the robot climb 
raise lift 
lower lift 
*/

package frc.team166.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.chopshoplib.commands.ActionCommand;
import frc.team166.chopshoplib.commands.CommandChain;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.chopshoplib.sensors.Lidar;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;
import frc.team166.robot.RobotMap.PreferenceStrings;
import edu.wpi.first.wpilibj.DigitalInput;

public class Lift extends PIDSubsystem {
    // Defines Limit Switches (Digital imputs)
    DigitalInput bottomLimitSwitch = new DigitalInput(RobotMap.DigitalInputs.LIFT_LIMIT_SWITCH_BOTTOM);
    DigitalInput topLimitSwitch = new DigitalInput(RobotMap.DigitalInputs.LIFT_LIMIT_SWITCH_TOP);
    // Defines Encoders and sets the the distance per Tick
    Encoder liftEncoder = new Encoder(RobotMap.Encoders.LIFT_A, RobotMap.Encoders.LIFT_B);
    // This is for one inch
    private final double encoderDistancePerTick = 0.01636;
    // Defines Motors 
    WPI_VictorSPX liftMotorA = new WPI_VictorSPX(RobotMap.CAN.LIFT_MOTOR_A);
    WPI_VictorSPX liftMotorB = new WPI_VictorSPX(RobotMap.CAN.LIFT_MOTOR_B);
    // Defines the previus motors as one motor 
    SpeedControllerGroup liftDrive = new SpeedControllerGroup(liftMotorA, liftMotorB);
    DoubleSolenoid liftBrake = new DoubleSolenoid(RobotMap.Solenoids.LIFT_BRAKE_A, RobotMap.Solenoids.LIFT_BRAKE_B);
    DoubleSolenoid liftTransmission = new DoubleSolenoid(RobotMap.Solenoids.LIFT_TRANSMISSION_A,
            RobotMap.Solenoids.LIFT_TRANSMISSION_B);

    //TODO we need to calculate these
    //these define the PID values for the lift
    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;
    private static double kF = 0;

    //this adds the LIDAR sensor
    private Lidar liftLidar = new Lidar(Port.kOnboard, 0x60);

    //enumerator that will be pulled from for the GoToHeight Command
    public enum LiftHeights {
        //will be changed
        kFloor(0), kSwitch(1), kPortal(2), kIntake(3), kScaleLow(4), kScaleHigh(5), kClimb(6), kMaxHeight(7);

        private double value;

        LiftHeights(double value) {
            this.value = value;
        }

        public double get() {
            return value;
        }
    }

    //sets the maximum lidar distance before switching to the encoder
    private final static double kMaxLidarDistance = 60;

    public Lift() {
        super("Lift", kP, kI, kD, kF);
        setOutputRange(Preferences.getInstance().getDouble(PreferenceStrings.DOWN_MAX_SPEED, -1),
                Preferences.getInstance().getDouble(PreferenceStrings.UP_MAX_SPEED, 1));
        setAbsoluteTolerance(0.05);
        liftEncoder.setDistancePerPulse(encoderDistancePerTick);
        //creates a child for the encoders and other stuff (limit switches, lidar, etc.)
        addChild(liftEncoder);
        addChild(topLimitSwitch);
        addChild(bottomLimitSwitch);
        addChild(liftLidar);
        addChild(findLiftHeight());
        liftDrive.setInverted(true);

        PreferenceStrings.setDefaultDouble(PreferenceStrings.LIFT_UP_DOWN_INCREMENT, 1);
        PreferenceStrings.setDefaultDouble(PreferenceStrings.UP_MAX_SPEED, 1);
        PreferenceStrings.setDefaultDouble(PreferenceStrings.DOWN_MAX_SPEED, 1);
        PreferenceStrings.setDefaultBool(PreferenceStrings.USE_LIDAR, false);
        PreferenceStrings.setDefaultDouble(PreferenceStrings.LIFT_CYCLES_BEFORE_STOP, 1);

        registerCommands();
    }

    private void registerCommands() {
        SmartDashboard.putData("Brake", Brake());
        SmartDashboard.putData("Shift to Low Gear", ShiftToLowGear());
        SmartDashboard.putData("Shift to High Gear", ShiftToHighGear());
    }

    protected double returnPIDInput() {
        return findLiftHeight();
    }

    private void brake() {
        liftBrake.set(Value.kForward);
    }

    private void disengageBrake() {
        liftBrake.set(Value.kReverse);
    }

    protected void usePIDOutput(double output) {
        if (topLimitSwitch.get() == true && output > 0) {
            setSetpoint(LiftHeights.kMaxHeight.get());
            liftDrive.stopMotor();
            return;
        }
        if (bottomLimitSwitch.get() == true && output < 0) {
            liftEncoder.reset();
            setSetpoint(LiftHeights.kFloor.get());
            liftDrive.stopMotor();
            return;
        }
        liftDrive.set(output);
    }

    public void reset() {
        liftDrive.stopMotor();
    }

    public double findLiftHeight() {
        if (Preferences.getInstance().getBoolean(PreferenceStrings.USE_LIDAR, false) == true) {
            if (liftLidar.getDistance(true) > kMaxLidarDistance) {
                return (liftLidar.getDistance(true));
            } else {
                return (liftEncoder.getDistance());
            }
        } else {
            return (liftEncoder.getDistance());
        }
    }

    //gear changes
    public void shiftToHighGear() {
        liftTransmission.set(Value.kReverse);
    }

    public void shiftToLowGear() {
        liftTransmission.set(Value.kForward);
    }

    //does not do anything
    public void initDefaultCommand() {
        setDefaultCommand(ManualLift());
    }

    public Command GoToHeight(LiftHeights height, boolean isHighGear) {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                disengageBrake();
                if (isHighGear == true) {
                    shiftToHighGear();
                } else {
                    shiftToLowGear();
                }
                setSetpoint(height.get());
            }

            @Override
            protected boolean isFinished() {
                return true;
            }
        };
    }

    public Command ManualLift() {
        return new SubsystemCommand(this) {
            double topLimitSwitchCounter = 0;
            double bottomLimitSwitchCounter = 0;
            Boolean downStop = false;
            Boolean upStop = false;

            @Override
            protected void initialize() {
                disengageBrake();
                disable();
            }

            @Override
            protected void execute() {
                double elevatorControl = Robot.m_oi.xBoxTempest.getTriggerAxis(Hand.kRight)
                        - Robot.m_oi.xBoxTempest.getTriggerAxis(Hand.kLeft);

                if (elevatorControl >= 0 && topLimitSwitch.get()) {
                    if (topLimitSwitchCounter < Preferences.getInstance()
                            .getDouble(PreferenceStrings.LIFT_CYCLES_BEFORE_STOP, 2)) {
                        topLimitSwitchCounter++;
                    } else {
                        upStop = true;
                    }
                }

                if (!topLimitSwitch.get()) {
                    if (topLimitSwitchCounter > 0) {
                        topLimitSwitchCounter--;
                    } else {
                        upStop = false;
                    }
                }
                if (upStop == true) {
                    liftDrive.set(Robot.m_oi.xBoxTempest.getTriggerAxis(Hand.kLeft));
                    return;
                }

                if (elevatorControl <= 0 && bottomLimitSwitch.get()) {
                    if (bottomLimitSwitchCounter < Preferences.getInstance()
                            .getDouble(PreferenceStrings.LIFT_CYCLES_BEFORE_STOP, 2)) {
                        bottomLimitSwitchCounter++;
                    } else {
                        downStop = true;
                        liftEncoder.reset();
                    }
                }

                if (!bottomLimitSwitch.get()) {
                    if (bottomLimitSwitchCounter > 0) {
                        bottomLimitSwitchCounter--;
                    } else {
                        downStop = false;
                    }
                }
                if (downStop == true) {
                    liftDrive.set(Robot.m_oi.xBoxTempest.getTriggerAxis(Hand.kRight));
                    return;
                }

                liftDrive.set(elevatorControl);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                enable();
            }

            @Override
            protected void interrupted() {
                enable();
            }
        };
    }

    public Command GoUp() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                disengageBrake();
            }

            @Override
            protected void execute() {
                setSetpointRelative(Preferences.getInstance().getDouble(PreferenceStrings.LIFT_UP_DOWN_INCREMENT, 1));
            }

            @Override
            protected boolean isFinished() {
                return false;
            }
        };
    }

    public Command GoDown() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                disengageBrake();
            }

            @Override
            protected void execute() {
                setSetpointRelative(-Preferences.getInstance().getDouble(PreferenceStrings.LIFT_UP_DOWN_INCREMENT, 1));

            }

            @Override
            protected boolean isFinished() {
                return false;
            }
        };
    }

    public Command ClimbUp() {
        return new CommandChain("Climb Up").then(DisengageBrake()).then(ShiftToHighGear())
                .then(GoToHeight(LiftHeights.kClimb, true)).then(ShiftToLowGear())
                .then(GoToHeight(LiftHeights.kScaleLow, false)).then(Brake());
    }

    public Command ShiftToHighGear() {
        return new ActionCommand("Shift To High Gear", this, this::shiftToHighGear);
    }

    public Command ShiftToLowGear() {
        return new ActionCommand("Shift To Low Gear", this, this::shiftToLowGear);
    }

    public Command Brake() {
        return new ActionCommand("Brake", this, this::brake);
    }

    public Command DisengageBrake() {
        return new ActionCommand("Don't Brake", this, this::disengageBrake);
    }
}
