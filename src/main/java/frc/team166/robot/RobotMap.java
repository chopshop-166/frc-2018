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
        public final static int frontright = 1;
        public final static int frontleft = 3;
        public final static int backright = 4;
        public final static int backleft = 2;
        public final static int rollerLeft = 5;
        public final static int rollerRight = 6;
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
        public final static int ir = 2;
    }

    public static class Buttons {
        //changes button ports into integers
        public final static int XboxAbutton = 1;
        public final static int XboxXbutton = 3;
    }

    public static class Encoders {
        public static final int leftRollerA = 4;
        public static final int rightRollerA = 5;
        public static final int leftRollerB = 6;
        public static final int rightRollerB = 7;
    }

    public static class Solenoids {
        public static final int manipulatorSolenoidA = 1;
        public static final int manipulatorSolenoidB = 2;
    }
}