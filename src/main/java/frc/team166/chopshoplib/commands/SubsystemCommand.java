package frc.team166.chopshoplib.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * A command that relies on a subsystem
 */
public abstract class SubsystemCommand extends Command {
    /**
     * Create a command that depends on a system
     * @param subsystem The subsystem to depend on
     */
    public SubsystemCommand(Subsystem subsystem) {
        setSubsystem(subsystem.getName());
        requires(subsystem);
    }

    /**
     * Create a command that depends on a system
     * @param name The name of the command
     * @param subsystem The subsystem to depend on
     */
    public SubsystemCommand(String name, Subsystem subsystem) {
        this(subsystem);
        setName(name);
    }
}
