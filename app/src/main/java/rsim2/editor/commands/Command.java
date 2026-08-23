package rsim2.editor.commands;

public interface Command {
    void execute();
    void undo();
    default String getDescription() {
        return getClass().getSimpleName();
    }
}
