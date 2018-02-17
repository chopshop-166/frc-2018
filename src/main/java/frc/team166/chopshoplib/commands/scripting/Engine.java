package frc.team166.chopshoplib.commands.scripting;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.command.Command;

public interface Engine {
    /**
     * Register a command function with the given prefix
     * @param prefix The prefix for use in scripts
     * @param func The function that creates the given command, given a double parameter
     */
    void registerHandler(String prefix, Function<String, Command> func);

    /**
     * Register a command function with the given prefix
     * @param prefix The prefix for use in scripts
     * @param func The function that creates the given command, given a double parameter
     */
    default void register(String prefix, Function<Double, Command> func) {
        registerHandler(prefix, s -> {
            double arg = Double.parseDouble(s);
            return func.apply(arg);
        });
    }

    /**
     * Register a command function with the given prefix
     * @param prefix The prefix for use in scripts
     * @param func The function that creates the given command
     */
    default void register(String prefix, Supplier<Command> func) {
        registerHandler(prefix, s -> func.get());
    }

    /**
     * Register a command function with the given prefix
     * @param prefix The prefix for use in scripts
     * @param func The function that creates the given command
     */
    default void register(Scriptable s) {
        s.registerScriptable(this);
    }

    /**
     * Unregister a command function with the given prefix
     *
     * If no new command is specified for this prefix, its usage in scripts will be an error
     * @param prefix The prefix for use in scripts
     */
    void unregister(String prefix);

    /**
     * Create a sequence of commands from the provided script
     * @param script The text of the script to translate
     */
    Command parseScript(String script);
}