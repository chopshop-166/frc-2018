/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    public static class CAN {
        //changes motor ports into integers
        public final static int FRONT_RIGHT = 1;
        public final static int FRONT_LEFT = 3;
        public final static int BACK_RIGHT = 4;
        public final static int BACK_LEFT = 2;
        public final static int ROLLER_LEFT = 5;
        public final static int ROLLER_RIGHT = 6;
    }

    public static class Controller {
        //changes controller ports into integers
        public final static int leftcontrol = 0;
        public final static int rightcontrol = 1;
        public final static int Xboxcontrol = 2;
    }

    public static class AnalogInputs {
        //changes input ports into integers
        public final static int tempestgyro = 1;
        public final static int IR = 2;
    }

    public static class Buttons {
        //changes button ports into integers
        public final static int XboxAbutton = 1;
        public final static int XboxXbutton = 3;
    }

    public static class Encoders {
    }

    public static class Solenoids {
        public static final int MANIPULATOR_SOLENOID_A = 1;
        public static final int MANIPULATOR_SOLENOID_B = 2;
    }

    public static class Preferences {
        public static final String MANIPULATOR_MOTOR_INTAKE_SPEED = "manipulatorMotorIntakeSpeed";
        public static final String MANIPULATOR_MOTOR_DISCHARGE_SPEED = "manipulatorMotorDischargeSpeed";
        public static final String CUBE_PICKUP_DISTANCE = "cubePickupDistance";
        public static final String CUBE_EJECT_WAIT_TIME = "cubeEjectWaitTime";
    }
}