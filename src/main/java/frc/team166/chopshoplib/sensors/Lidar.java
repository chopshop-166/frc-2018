package frc.team166.chopshoplib.sensors;

import java.nio.ByteBuffer;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class Lidar extends SensorBase {
    I2C i2cDevice;
    Thread t;
    private int distanceMM;

    private class PollSensor implements Runnable {
        public void run() {
            while (true) {
                /* Get the distance from the sensor */
                readDistance();
                /* Sensor updates at 60Hz, but we'll run this at 50 since the math is nicer */
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    /* We stop for nothing! */
                }
            }
        }
    }

    /**
     * Create a LIDAR object
     * 
     * @param port The I2C port the sensor is connected to
     * @param kAddress The I2C address the sensor is found at
     */
    public Lidar(Port port, int kAddress) {
        i2cDevice = new I2C(port, kAddress);
        setName("Lidar", kAddress);
        t = new Thread(new PollSensor());
    }

    /**
     * This function gets the distance from a LiDAR sensor
     * @param bFlag True requests the distance in inches, false requests the distance in mm
     */
    public double getDistance(Boolean bFlag) {
        if (bFlag == true) {
            return (distanceMM / 25.4);
        } else {
            return distanceMM;
        }
    }

    private void readDistance() {
        byte[] dataBuffer = new byte[2];

        i2cDevice.write(0x44, 0x1);
        i2cDevice.readOnly(dataBuffer, 2);
        ByteBuffer bbConvert = ByteBuffer.wrap(dataBuffer);
        distanceMM = bbConvert.getShort();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("LiDAR");
        NetworkTableEntry mmDistance = builder.getEntry("Distance");
        builder.setUpdateTable(() -> {
            mmDistance.setDouble(getDistance(true));
        });
    }

}