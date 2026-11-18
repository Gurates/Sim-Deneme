package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCond;
import imgui.type.ImBoolean;
import rsim2.editor.SelectionManager;
import rsim2.scene.ObjectInteractionController;
import rsim2.scene.SceneNode;

import java.util.List;

public class SceneObjectHierarchy {
    private final ImBoolean open = new ImBoolean(false);

    public void toggleVisible() {
        open.set(!open.get());
    }

    public void render(SceneNode rootNode, SelectionManager selectionManager) {
        if (!open.get()) return;
        ImGui.setNextWindowSize(320, 360, ImGuiCond.FirstUseEver);
        if (ImGui.begin("Scene Object Hierarchy", open)) {
            List<SceneNode> objects = ObjectInteractionController.objects(rootNode);
            if (objects.isEmpty()) {
                ImGui.textDisabled("No scene objects. Use Import Object to add one.");
            }
            for (int i = 0; i < objects.size(); i++) {
                SceneNode object = objects.get(i);
                boolean selected = selectionManager != null && selectionManager.getSelected() == object;
                if (ImGui.selectable(object.getId() + "##sceneObject" + i, selected)
                        && selectionManager != null) {
                    selectionManager.select(object);
                }
            }
        }
        ImGui.end();
    }
}
