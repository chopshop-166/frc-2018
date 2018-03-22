package frc.team166.chopshoplib.outputs;

import edu.wpi.first.wpilibj.DigitalOutput;

public class DigitalOutputDutyCycle extends DigitalOutput {

    private double mRate = 0;

    public DigitalOutputDutyCycle(final int channel) {
        super(channel);
    }

    @Override
    public void updateDutyCycle(double rate) {
        mRate = rate;
        super.updateDutyCycle(rate);
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