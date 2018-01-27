package frc.team166.chopshoplib.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 * A declarative command sequence class.
 * Allows creating command groups by chaining calls to .then()
 */
public class CommandChain extends CommandGroup {
    //#region Constructors
    /**
     * Create a CommandChain
     */
    public CommandChain() {
    }

    /**
     * Create a CommandChain with the given name
     * @param name The name of the command chain
     */
    public CommandChain(String name) {
        super(name);
    }

    /**
     * Create a CommandChain preloaded with commands to be run in parallel
     * @param cmds The first commands to run
     */
    public CommandChain(Command... cmds) {
        addCommands(cmds);
    }

    /**
     * Create a CommandChain with a name and commands to run
     * @param name The name of the command chain
     * @param cmds The first commands to run
     */
    public CommandChain(String name, Command... cmds) {
        super(name);
        addCommands(cmds);
    }
    //#endregion

    //#region Then-commands
    /**
     * Do a set of commands after the ones already provided
     * @param cmds The commands to run next
     */
    public CommandChain then(Command... cmds) {
        addCommands(cmds);
        return this;
    }

    /**
     * Do a set of commands after the ones already provided, with a timeout
     * @param timeout The maximum amount of time before moving on to the next commands
     * @param cmds The commands to run next
     */
    public CommandChain then(double timeout, Command... cmds) {
        addCommands(timeout, cmds);
        return this;
    }
    //#endregion

    /**
     * Add all provided commands as a group
     * @param cmds The commands to run next
     */
    private void addCommands(Command... cmds) {
        if (cmds.length == 1) {
            addSequential(cmds[0]);
        } else {
            for (Command c : cmds) {
                addParallel(c);
            }
            addSequential(new WaitForChildren());
        }
    }

    /**
     * Add all provided commands as a group with a timeout
     * @param timeout The maximum amount of time before moving on to the next commands
     * @param cmds The commands to run next
     */
    private void addCommands(double timeout, Command... cmds) {
        if (cmds.length == 1) {
            addSequential(cmds[0], timeout);
        } else {
            for (Command c : cmds) {
                addParallel(c, timeout);
            }
            addSequential(new WaitForChildren());
        }
    }
}
