package frc.team166.robot;

import edu.wpi.first.wpilibj.PIDOutput;

public class PIDOutputVariable implements PIDOutput {

    //store value for the PIDOutput under Drive.java

    private double value;

    @Override
    public void pidWrite(double output) {
        value = output;
    }

    public double get() {
        return value;
    }
}