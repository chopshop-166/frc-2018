package frc.team166.chopshoplib.commands.scripting;

import java.util.Arrays;
import java.util.HashMap;
import java.util.function.Function;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.team166.chopshoplib.commands.CommandChain;
import frc.team166.chopshoplib.commands.TimeoutCommand;

/**
 * A simple language meant for creating series of commands
 * Sets of commands are separated with semicolons
 * Commands to be run at the same time are separated by pipes
 * Command names are separated from their optional double arguments by whitespace
 * A command can be given a timeout with `timeout 5 mycommand`
 */
public class SimpleEngine implements Engine {
    HashMap<String, Function<String, Command>> handlers = new HashMap<>();

    /**
     * Initialize the default handlers
     */
    public SimpleEngine() {
        register("wait", WaitCommand::new);
        registerHandler("print", PrintCommand::new);
    }

    /**
     * Register a command function with the given prefix
     * @param prefix The prefix for use in scripts
     * @param func The function that creates the given command, given a double parameter
     */
    public void registerHandler(String prefix, Function<String, Command> func) {
        handlers.put(prefix, func);
    }

    /**
     * Unregister a command function with the given prefix
     *
     * If no new command is specified for this prefix, its usage in scripts will be an error
     * @param prefix The prefix for use in scripts
     */
    public void unregister(String prefix) {
        handlers.remove(prefix);
    }

    /**
     * Parse the entire script into a command group
     */
    public Command parseScript(String script) {
        CommandChain result = new CommandChain(script);
        if (!"".equals(script)) {
            for (String groupStr : script.trim().split(";")) {
                Command[] cmds = Arrays.stream(groupStr.split("\\|")).map(String::trim).map(this::parseSingleCommand)
                        .toArray(Command[]::new);
                result.then(cmds);
            }
        }
        return result;
    }

    /**
     * Create a command from a string
     * @param cmd The name of the command to look up
     */
    Command parseSingleCommand(String cmd) {
        String[] args = cmd.split("[\\s,]+");
        if (args.length == 0) {
            // Something is terribly wrong
            return null;
        }

        if (args.length > 2 && args[0].equals("timeout")) {
            double timeoutLength = Double.parseDouble(args[1]);
            String[] cmdargs = new String[args.length - 2];
            for (int i = 0; i < args.length - 2; i++) {
                cmdargs[i] = args[i + 2];
            }
            Command wrapped = parseArgs(cmdargs);
            if (wrapped != null) {
                return new TimeoutCommand(wrapped, timeoutLength);
            }
        } else {
            return parseArgs(args);
        }
        return null;
    }

    /**
     * Create a command from a string
     * @param args The split arguments, including command name
     */
    Command parseArgs(String[] args) {
        Function<String, Command> constructor = handlers.get(args[0]);
        if (constructor != null) {
            String argument = "";
            if (args.length > 1) {
                argument = args[1];
            }
            return constructor.apply(argument);
        } else {
            return null;
        }
    }
}
