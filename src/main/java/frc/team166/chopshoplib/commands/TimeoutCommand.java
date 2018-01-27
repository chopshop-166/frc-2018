package frc.team166.chopshoplib.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * A command that runs another command.
 * The command will be killed after a certain amount of time,
 * if it hasn't already run to completion.
 */
public class TimeoutCommand extends TimedCommand {
    /**
     * Wrap the provided command with a timeout
     * @param cmd The command to time out
     * @param timeout The maximum time before timing out
     */
    public TimeoutCommand(Command cmd, double timeout) {
        this("Timeout(" + cmd.getName() + ", " + timeout + ")", cmd, timeout);
    }

    /**
     * Wrap the provided command with a timeout
     * @param name The name for the timed out command
     * @param cmd The command to time out
     * @param timeout The maximum time before timing out
     */
    public TimeoutCommand(String name, Command cmd, double timeout) {
        super(name, timeout);
        m_command = cmd;
    }

    @Override
    protected void initialize() {
        m_command.start();
    }

    @Override
    protected void end() {
        if (m_command.isRunning()) {
            m_command.cancel();
        }
    }

    @Override
    protected void interrupted() {
        if (m_command.isRunning()) {
            m_command.cancel();
        }
    }

    Command m_command = null;
}
