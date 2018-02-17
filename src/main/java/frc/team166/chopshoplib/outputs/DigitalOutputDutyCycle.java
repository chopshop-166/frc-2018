package frc.team166.chopshoplib.outputs;

import edu.wpi.first.wpilibj.DigitalOutput;

public class DigitalOutputDutyCycle extends DigitalOutput {

    private double mRate = 0;

    public DigitalOutputDutyCycle(final int channel) {
        super(channel);
    }

    @Override
    public void setPWMRate(double rate) {
        mRate = rate;
        super.setPWMRate(rate);
    }

    @Override
    public void enablePWM(double initialDutyCycle) {
        mRate = initialDutyCycle;
        super.enablePWM(initialDutyCycle);
    }

    public double getPWMRate() {
        return mRate;
    }
}