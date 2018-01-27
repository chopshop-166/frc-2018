package frc.team166.chopshoplib.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * A declarative command class.
 * Usable with a Runnable to create commands inside the subsystems
 */
public class ActionCommand extends InstantCommand {
    /**
     * Create a command that calls the given action when run
     * @param action The action to take when the command is run
     */
    public ActionCommand(Runnable action) {
        m_action = action;
    }

    /**
     * Create a command that depends on the given subsystem and calls
     * the provided action when run
     * @param subsystem The subsystem that the command depends on
     * @param action The action to take when the command is run
     */
    public ActionCommand(Subsystem subsystem, Runnable action) {
        useSubsystem(subsystem);
        m_action = action;
    }

    /**
     * Create a named command that calls the given action when run
     * @param name The name of the command
     * @param action The action to take when the command is run
     */
    public ActionCommand(String name, Runnable action) {
        super(name);
        m_action = action;
    }

    /**
     * Create a named command that depends on the given subsystem and calls
     * the provided action when run
     * @param name The name of the command
     * @param subsystem The subsystem that the command depends on
     * @param action The action to take when the command is run
     */
    public ActionCommand(String name, Subsystem subsystem, Runnable action) {
        super(name);
        useSubsystem(subsystem);
        m_action = action;
    }

    /**
     * Trigger the stored action.
     * Called just before this Command runs the first time.
     */
    @Override
    protected void initialize() {
        if (m_action != null) {
            m_action.run();
        }
    }

    /**
     * Use the given subsystem's name and mark it required
     * @param subsystem The subsystem to depend on
     */
    private void useSubsystem(Subsystem subsystem) {
        setSubsystem(subsystem.getName());
        requires(subsystem);
    }

    Runnable m_action = null;
}
