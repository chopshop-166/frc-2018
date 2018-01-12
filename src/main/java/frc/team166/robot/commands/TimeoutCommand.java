package frc.team166.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * A command that runs another command, killing it after a certain amount of time
 */
public class TimeoutCommand extends TimedCommand {
    /**
     * Wrap the provided command with a timeout
     * @param cmd The command to time out
     * @param timeout The maximum time before timing out
     */
    public TimeoutCommand(Command cmd, double timeout) {
        super(timeout);
        setName("Timeout(" + cmd.getName() + ", " + timeout + ")");
        m_command = cmd;
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
        if(m_command.isRunning())
        {
            m_command.cancel();
        }
    }

    @Override
    protected void interrupted() {
        if(m_command.isRunning())
        {
            m_command.cancel();
        }
    }

    Command m_command = null;
}
