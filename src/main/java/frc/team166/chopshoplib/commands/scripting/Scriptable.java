package frc.team166.chopshoplib.commands.scripting;

/**
 * An object that can be tied into a scripting engine
 */
@FunctionalInterface
public interface Scriptable {

    void registerScriptable(Engine e);

}
