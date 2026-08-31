package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import rsim2.collision.CollisionFilter;
import rsim2.collision.CollisionResult;
import rsim2.collision.CollisionWorld;
import rsim2.collision.ContactPair;
import rsim2.core.Engine;
import rsim2.editor.SelectionManager;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class CollisionPanel {
    private final Engine engine;
    private SceneNode rootNode;
    private List<Joint> joints;
    private final SelectionManager selectionManager;
    private final CollisionWorld collisionWorld;

    private boolean visible = false;

    private final ImBoolean enableCollision = new ImBoolean(true);
    private final ImBoolean enableSelfCollision = new ImBoolean(true);
    private final ImBoolean enableGroundCollision = new ImBoolean(true);
    private final ImBoolean ignoreAdjacent = new ImBoolean(true);
    private final ImBoolean debugWireframes = new ImBoolean(false);
    private final ImBoolean stopOnCollision = new ImBoolean(true);
    private final ImFloat groundHeight = new ImFloat(0.0f);

    private final List<String> trajectoryValidationLogs = new ArrayList<>();
    private boolean trajectoryValidated = false;
    private boolean trajectoryPassed = false;

    public CollisionPanel(Engine engine, SceneNode rootNode, List<Joint> joints,
                          SelectionManager selectionManager, CollisionWorld collisionWorld) {
        this.engine = engine;
        this.rootNode = rootNode;
        this.joints = joints != null ? joints : new ArrayList<>();
        this.selectionManager = selectionManager;
        this.collisionWorld = collisionWorld;
    }

    public void setRootNode(SceneNode rootNode) {
        this.rootNode = rootNode;
    }

    public void setJoints(List<Joint> joints) {
        this.joints = joints != null ? joints : new ArrayList<>();
    }

    public boolean isVisible() {
        return visible;
    }

    public void setVisible(boolean visible) {
        this.visible = visible;
    }

    public void toggleVisible() {
        this.visible = !this.visible;
    }

    public boolean isStopOnCollision() {
        return stopOnCollision.get();
    }

    public void render() {
        if (!visible) return;

        float displayWidth = ImGui.getIO().getDisplaySizeX();
        float displayHeight = ImGui.getIO().getDisplaySizeY();
        float panelWidth = 320.0f;
        float panelHeight = Math.min(380.0f, displayHeight - 80.0f);

        ImGui.setNextWindowPos(displayWidth - 320.0f - panelWidth - 10.0f, 50.0f, ImGuiCond.FirstUseEver);
        ImGui.setNextWindowSize(panelWidth, panelHeight, ImGuiCond.FirstUseEver);

        ImBoolean openBool = new ImBoolean(visible);
        if (ImGui.begin("Collision Detection", openBool, ImGuiWindowFlags.None)) {

            CollisionFilter filter = collisionWorld.getFilter();
            CollisionResult result = collisionWorld.getLastResult();

            enableCollision.set(filter.isEnabled());
            enableGroundCollision.set(filter.isGroundCollisionEnabled());
            debugWireframes.set(collisionWorld.isDebugWireframesEnabled());

            if (!filter.isEnabled()) {
                ImGui.pushStyleColor(ImGuiCol.Button, 0.35f, 0.35f, 0.35f, 1.0f);
                ImGui.button("Disabled", -1.0f, 26.0f);
                ImGui.popStyleColor();
            } else if (result.hasCollision()) {
                ImGui.pushStyleColor(ImGuiCol.Button, 0.85f, 0.2f, 0.2f, 1.0f);
                ImGui.button(String.format("🔴 %d Collisions Detected", result.getCollisionCount()), -1.0f, 26.0f);
                ImGui.popStyleColor();
            } else {
                ImGui.pushStyleColor(ImGuiCol.Button, 0.15f, 0.65f, 0.35f, 1.0f);
                ImGui.button("🟢 Safe (No Collision)", -1.0f, 26.0f);
                ImGui.popStyleColor();
            }

            ImGui.spacing();

            if (ImGui.checkbox("Enable Collision Detection", enableCollision)) {
                filter.setEnabled(enableCollision.get());
            }

            if (ImGui.checkbox("Ground Collision", enableGroundCollision)) {
                filter.setGroundCollisionEnabled(enableGroundCollision.get());
            }

            if (ImGui.checkbox("Show 3D Bounding Boxes (Wireframe)", debugWireframes)) {
                collisionWorld.setDebugWireframesEnabled(debugWireframes.get());
            }

            ImGui.separator();
            ImGui.spacing();

            if (result.hasCollision() && filter.isEnabled()) {
                ImGui.textColored(1.0f, 0.4f, 0.4f, 1.0f, "Colliding Parts:");

                if (ImGui.button("Ignore Current Contacts", -1.0f, 24.0f)) {
                    filter.ignoreCurrentContacts(result);
                }
                ImGui.spacing();

                List<ContactPair> contacts = result.getContacts();
                for (int i = 0; i < contacts.size(); i++) {
                    ContactPair contact = contacts.get(i);
                    ImGui.pushID("contact_" + i);

                    ImGui.bulletText(contact.getDescription());

                    ImGui.sameLine(ImGui.getWindowWidth() - 75.0f);
                    if (contact.getNodeA() != null && contact.getNodeB() != null) {
                        if (ImGui.button("Ignore", 60.0f, 19.0f)) {
                            filter.ignorePair(contact.getNodeA().getId(), contact.getNodeB().getId());
                        }
                    }
                    ImGui.popID();
                }
                ImGui.spacing();
                ImGui.separator();
            }

            Set<String> ignoredPairs = filter.getCustomIgnoredPairs();
            if (!ignoredPairs.isEmpty()) {
                if (ImGui.collapsingHeader("Ignored Pairs (" + ignoredPairs.size() + ")")) {
                    if (ImGui.button("Clear All", -1.0f, 22.0f)) {
                        filter.clearIgnoredPairs();
                    }
                    ImGui.spacing();
                    String toRemove = null;
                    for (String pair : ignoredPairs) {
                        ImGui.bulletText(pair);
                        ImGui.sameLine(ImGui.getWindowWidth() - 40.0f);
                        if (ImGui.button("X##" + pair, 24.0f, 18.0f)) {
                            toRemove = pair;
                        }
                    }
                    if (toRemove != null) {
                        String[] parts = toRemove.split("::");
                        if (parts.length == 2) filter.unignorePair(parts[0], parts[1]);
                    }
                }
            }

        }
        ImGui.end();

        if (!openBool.get()) {
            this.visible = false;
        }
    }
}
