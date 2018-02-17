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

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.team166.chopshoplib.commands.ActionCommand;
import frc.team166.chopshoplib.commands.CommandChain;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.chopshoplib.sensors.Lidar;
import frc.team166.robot.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;

public class Lift extends PIDSubsystem {
    // Defines Limit Switches (Digital imputs)
    DigitalInput bottomLimitSwitch = new DigitalInput(RobotMap.DigitalInputs.LIFT_LIMIT_SWITCH_BOTTOM);
    DigitalInput topLimitSwitch = new DigitalInput(RobotMap.DigitalInputs.LIFT_LIMIT_SWITCH_TOP);
    // Defines Encoders 
    Encoder liftEncoder = new Encoder(RobotMap.Encoders.LIFT_A, RobotMap.Encoders.LIFT_B);
    // TODO we still need to calculate this
    private final double encoderDistancePerTick = 0.01;
    // Defines Motors 
    WPI_TalonSRX liftMotorA = new WPI_TalonSRX(RobotMap.CAN.LIFT_MOTOR_A);
    WPI_TalonSRX liftMotorB = new WPI_TalonSRX(RobotMap.CAN.LIFT_MOTOR_B);
    // Defines the previus motors as one motor 
    SpeedControllerGroup liftDrive = new SpeedControllerGroup(liftMotorA, liftMotorB);
    DoubleSolenoid liftBrake = new DoubleSolenoid(RobotMap.Solenoids.LIFT_BRAKE_A, RobotMap.Solenoids.LIFT_BRAKE_B);
    DoubleSolenoid liftTransmission = new DoubleSolenoid(RobotMap.Solenoids.LIFT_TRANSMISSION_A,
            RobotMap.Solenoids.LIFT_TRANSMISSION_B);

    private static double kP = Preferences.getInstance().getDouble(RobotMap.Preferences.K_P, 1);
    private static double kI = Preferences.getInstance().getDouble(RobotMap.Preferences.K_I, 1);
    private static double kD = Preferences.getInstance().getDouble(RobotMap.Preferences.K_D, 1);
    private static double kF = Preferences.getInstance().getDouble(RobotMap.Preferences.K_F, 1);

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

    private final static double kMaxLidarDistance = 60;

    public Lift() {
        super("Lift", kP, kI, kD, kF);
        setOutputRange(Preferences.getInstance().getDouble(RobotMap.Preferences.DOWN_MAX_SPEED, -1),
                Preferences.getInstance().getDouble(RobotMap.Preferences.UP_MAX_SPEED, 1));
        setAbsoluteTolerance(0.05);
        liftEncoder.setDistancePerPulse(encoderDistancePerTick);
        //creates a child for the encoders and other stuff (limit switches, lidar, etc.)
        addChild(liftEncoder);
        addChild(topLimitSwitch);
        addChild(bottomLimitSwitch);
        addChild(liftLidar);
        addChild(findLiftHeight());

        Preferences.getInstance().getDouble(RobotMap.Preferences.K_P, 1);
        Preferences.getInstance().getDouble(RobotMap.Preferences.K_I, 1);
        Preferences.getInstance().getDouble(RobotMap.Preferences.K_D, 1);
        Preferences.getInstance().getDouble(RobotMap.Preferences.K_F, 1);
        Preferences.getInstance().getDouble(RobotMap.Preferences.LIFT_UP_DOWN_INCREMENT, 1);
        Preferences.getInstance().getDouble(RobotMap.Preferences.UP_MAX_SPEED, 1);
        Preferences.getInstance().getDouble(RobotMap.Preferences.DOWN_MAX_SPEED, 1);
        Preferences.getInstance().getBoolean(RobotMap.Preferences.USE_LIDAR, false);
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

    public double findLiftHeight() {
        if (Preferences.getInstance().getBoolean(RobotMap.Preferences.USE_LIDAR, false) == true) {
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
        liftTransmission.set(Value.kForward);
    }

    public void shiftToLowGear() {
        liftTransmission.set(Value.kReverse);
    }

    //does not do anything
    public void initDefaultCommand() {
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
            @Override
            protected void initialize() {
                disengageBrake();
                disable();
                liftDrive.set(0);
            }

            @Override
            protected void execute() {
            }

            @Override
            protected boolean isFinished() {
                if (topLimitSwitch.get() == true || bottomLimitSwitch.get() == true) {
                    return true;
                    else return false;
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
                setSetpointRelative(
                        Preferences.getInstance().getDouble(RobotMap.Preferences.LIFT_UP_DOWN_INCREMENT, 1));
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
                setSetpointRelative(
                        -Preferences.getInstance().getDouble(RobotMap.Preferences.LIFT_UP_DOWN_INCREMENT, 1));

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
