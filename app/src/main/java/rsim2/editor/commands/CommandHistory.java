package rsim2.editor.commands;

import java.util.ArrayDeque;
import java.util.Deque;

public class CommandHistory {
    private static final int MAX_HISTORY = 50;

    private final Deque<Command> undoStack = new ArrayDeque<>();
    private final Deque<Command> redoStack = new ArrayDeque<>();
    private final Runnable onStateChanged;

    public CommandHistory() {
        this(null);
    }

    public CommandHistory(Runnable onStateChanged) {
        this.onStateChanged = onStateChanged;
    }

    public void executeAndRecord(Command cmd) {
        if (cmd == null) return;
        cmd.execute();
        undoStack.push(cmd);
        if (undoStack.size() > MAX_HISTORY) {
            undoStack.removeLast();
        }
        redoStack.clear();
        if (onStateChanged != null) {
            onStateChanged.run();
        }
    }

    public void recordExecuted(Command cmd) {
        if (cmd == null) return;
        undoStack.push(cmd);
        if (undoStack.size() > MAX_HISTORY) {
            undoStack.removeLast();
        }
        redoStack.clear();
        if (onStateChanged != null) {
            onStateChanged.run();
        }
    }

    public void undo() {
        if (undoStack.isEmpty()) return;
        Command cmd = undoStack.pop();
        cmd.undo();
        redoStack.push(cmd);
        if (onStateChanged != null) {
            onStateChanged.run();
        }
    }

    public void redo() {
        if (redoStack.isEmpty()) return;
        Command cmd = redoStack.pop();
        cmd.execute();
        undoStack.push(cmd);
        if (onStateChanged != null) {
            onStateChanged.run();
        }
    }

    public boolean canUndo() {
        return !undoStack.isEmpty();
    }

    public boolean canRedo() {
        return !redoStack.isEmpty();
    }

    public void clear() {
        undoStack.clear();
        redoStack.clear();
    }
}
