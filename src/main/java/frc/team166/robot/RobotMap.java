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
		public final static int frontright = 1;
		public final static int frontleft = 3;
		public final static int backright = 4;
		public final static int backleft = 2;
	}

	public static class Controller {
		public final static int leftcontrol = 0;
		public final static int rightcontrol = 1;
	}

	public static class AnalogInputs {
		public final static int tempestgyro = 1;
	}
}