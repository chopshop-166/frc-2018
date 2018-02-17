package frc.team166.chopshoplib.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Button;
import java.util.Vector;

/**
 * Represents a joystick along with it's associated buttons
 * <p>
 * This class serves as a wrapper for a Joystick and all it's buttons.
 */
public class ButtonJoystick extends Joystick {
    Vector<Button> buttons = new Vector<Button>();

    /**
     * Construct an instance of a joystick along with each button the joystick has.
     *
     * @param port The USB port that the joystick is connected to on the Driver Station
     */
    public ButtonJoystick(int port) {
        super(port);
        // Just pad the vector to match the joysticks
        buttons.add(0, null);
        for (int i = 1; i <= this.getButtonCount(); i++) {
            buttons.add(i, new JoystickButton(this, i));
        }
    }

    /**
    * Get a button from this joystick
    * <p>
    * Returns the sepcified button of a joystick without having to explicitly create
    * each button.
    * 
    * @param buttonId The index of the button to accesss
    * @return The button object for the given ID
    */
    public Button getButton(int buttonId) {
        if (buttons.size() <= buttonId) {
            for (int i = buttons.size(); i <= buttonId; i++) {
                buttons.add(i, new JoystickButton(this, i));
            }
        }
        return buttons.get(buttonId);
    }
}