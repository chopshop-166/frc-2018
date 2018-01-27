package frc.team166.chopshoplib.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import java.util.Vector;

/**
 * Represents a joystick along with it's associated buttons
 * <p>
 * This class serves as a wrapper for a Joystick and all it's buttons.
 */
public class ButtonXboxController extends XboxController {
    Vector<JoystickButton> buttons = new Vector<JoystickButton>();

    /**
     * Construct an instance of an XboxController along with each button the xBox controller has.
     *
     * @param port The USB port that the xBox controller is connected to on the Driver Station
     */
    public ButtonXboxController(int port) {
        super(port);
        for (int i = 0; i < this.getButtonCount(); i++) {
            buttons.add(i, new JoystickButton(this, i));
        }
    }

    /**
        * Get a button from this xBox controller
        * <p>
        * Returns the sepcified button of a xBox controller without having to explicitly create
        * each button.
        * 
        * @param buttonId The index of the button to accesss
        * @return The button object for the given ID
        */
    public JoystickButton getButton(int buttonId) {
        try {
            return buttons.get(buttonId);
        } catch (ArrayIndexOutOfBoundsException e) {
            buttons.add(buttonId, new JoystickButton(this, buttonId));
            return buttons.get(buttonId);
        }
    }

    public enum xBoxButton {
        kBumperLeft(5), kBumperRight(6), kStickLeft(9), kStickRight(10), kA(1), kB(2), kX(3), kY(4), kBack(7), kStart(
                8);

        @SuppressWarnings("MemberName")
        private int value;

        xBoxButton(int value) {
            this.value = value;
        }
    }
}