package rsim2;

import org.lwjgl.system.JNI;
import org.lwjgl.system.Library;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.system.MemoryUtil;
import org.lwjgl.system.SharedLibrary;
import rsim2.core.Engine;

import java.nio.ByteBuffer;

public class App {
    public static void main(String[] args) {
        initWindowsAppId();
        new Engine().run();
    }

    public static void initWindowsAppId() {
        if (!System.getProperty("os.name").toLowerCase().contains("win")) {
            return;
        }
        try (MemoryStack stack = MemoryStack.stackPush()) {
            SharedLibrary shell32 = Library.loadNative("rsim2.App", "shell32");
            long func = shell32.getFunctionAddress("SetCurrentProcessExplicitAppUserModelID");
            if (func != 0) {
                ByteBuffer appID = stack.UTF16("RSim2.Simulator.App." + System.currentTimeMillis());
                JNI.callPI(MemoryUtil.memAddress(appID), func);
            }
        } catch (Throwable ignored) {
        }
    }
}
