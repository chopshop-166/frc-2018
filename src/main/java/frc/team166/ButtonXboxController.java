package frc.team166;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Button;
import java.util.Vector;

/**
 * Represents a joystick along with it's associated buttons
 * <p>
 * This class serves as a wrapper for a Joystick and all it's buttons.
 */
public class ButtonXboxController extends XboxController {
    Vector<Button> buttons = new Vector<Button>();

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
    public Button getButton(int buttonId) {
        return buttons.get(buttonId);
    }
}