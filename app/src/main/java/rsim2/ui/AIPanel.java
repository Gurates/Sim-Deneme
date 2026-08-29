package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiInputTextFlags;
import imgui.flag.ImGuiTreeNodeFlags;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import imgui.type.ImString;
import rsim2.ai.AiActionExecutor;
import rsim2.ai.AiConfig;
import rsim2.ai.AiService;
import rsim2.core.Engine;
import rsim2.editor.SelectionManager;
import rsim2.motion.MotionPlayer;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AIPanel {
    private boolean visible = false;
    private final ImBoolean isOpen = new ImBoolean(false);
    private boolean showSettings = false;
    private boolean showPassword = false;
    private boolean isLoading = false;

    private Engine engine;
    private SceneNode rootNode;
    private SelectionManager selectionManager;
    private List<Joint> joints;
    private MotionPlayer motionPlayer;

    private final AiConfig config;
    private final AiService aiService;

    private final ImString promptInput = new ImString(512);
    private final ImString geminiKeyInput = new ImString(256);
    private final ImString openAiKeyInput = new ImString(256);
    private final ImInt selectedProviderIdx = new ImInt(0);
    private final ImInt selectedModelIdx = new ImInt(0);

    private final List<ChatMessage> chatHistory = Collections.synchronizedList(new ArrayList<>());

    private static final String[] GEMINI_MODELS = { "gemini-3.6-flash", "gemini-2.5-flash", "gemini-1.5-flash", "gemini-1.5-pro" };
    private static final String[] OPENAI_MODELS = { "gpt-4o-mini", "gpt-4o", "gpt-3.5-turbo" };

    public static class ChatMessage {
        public String sender;
        public String text;
        public boolean isError;

        public ChatMessage(String sender, String text, boolean isError) {
            this.sender = sender;
            this.text = text;
            this.isError = isError;
        }
    }

    public AIPanel() {
        this(null, null, null, null, null);
    }

    public AIPanel(Engine engine, SceneNode rootNode, SelectionManager selectionManager, List<Joint> joints) {
        this(engine, rootNode, selectionManager, joints, null);
    }

    public AIPanel(Engine engine, SceneNode rootNode, SelectionManager selectionManager, List<Joint> joints, MotionPlayer motionPlayer) {
        this.engine = engine;
        this.rootNode = rootNode;
        this.selectionManager = selectionManager;
        this.joints = joints != null ? joints : new ArrayList<>();
        this.motionPlayer = motionPlayer != null ? motionPlayer : new MotionPlayer();

        this.config = AiConfig.load();
        this.aiService = new AiService();

        this.geminiKeyInput.set(config.getGeminiApiKey());
        this.openAiKeyInput.set(config.getOpenAiApiKey());
        this.selectedProviderIdx.set(config.getProvider() == AiConfig.Provider.GEMINI ? 0 : 1);

        syncModelIndexFromConfig();

        chatHistory.add(new ChatMessage("AI", "Hello! I am your Robot AI Assistant. You can ask me to generate motion trajectories (e.g., 'wave hand', 'bend elbow 45 degrees'), analyze kinematics, or explain robot properties.", false));

        if (config.getActiveApiKey().isEmpty()) {
            chatHistory.add(new ChatMessage("System", "No API Key entered yet. Please click the 'Settings' button above to configure your Google Gemini or OpenAI API key.", false));
        }
    }

    private void syncModelIndexFromConfig() {
        if (config.getProvider() == AiConfig.Provider.GEMINI) {
            String m = config.getGeminiModel();
            for (int i = 0; i < GEMINI_MODELS.length; i++) {
                if (GEMINI_MODELS[i].equalsIgnoreCase(m)) {
                    selectedModelIdx.set(i);
                    return;
                }
            }
            selectedModelIdx.set(0);
        } else {
            String m = config.getOpenAiModel();
            for (int i = 0; i < OPENAI_MODELS.length; i++) {
                if (OPENAI_MODELS[i].equalsIgnoreCase(m)) {
                    selectedModelIdx.set(i);
                    return;
                }
            }
            selectedModelIdx.set(0);
        }
    }

    public void setEngine(Engine engine) {
        this.engine = engine;
    }

    public void setRootNode(SceneNode rootNode) {
        this.rootNode = rootNode;
    }

    public void setSelectionManager(SelectionManager selectionManager) {
        this.selectionManager = selectionManager;
    }

    public void setJoints(List<Joint> joints) {
        this.joints = joints != null ? joints : new ArrayList<>();
    }

    public void setMotionPlayer(MotionPlayer motionPlayer) {
        this.motionPlayer = motionPlayer != null ? motionPlayer : new MotionPlayer();
    }

    public MotionPlayer getMotionPlayer() {
        return motionPlayer;
    }

    public boolean isVisible() {
        return visible;
    }

    public void setVisible(boolean visible) {
        this.visible = visible;
        this.isOpen.set(visible);
    }

    public void toggleVisible() {
        setVisible(!visible);
    }

    public void render() {
        if (!visible) {
            return;
        }

        float displayWidth = ImGui.getIO().getDisplaySizeX();
        float displayHeight = ImGui.getIO().getDisplaySizeY();
        float panelWidth = 430.0f;
        float panelHeight = 520.0f;

        ImGui.setNextWindowPos(displayWidth - 320.0f - panelWidth - 20.0f, 50.0f, ImGuiCond.FirstUseEver);
        ImGui.setNextWindowSize(panelWidth, panelHeight, ImGuiCond.FirstUseEver);

        isOpen.set(visible);
        if (ImGui.begin("AI Assistant", isOpen, ImGuiWindowFlags.None)) {

            ImGui.textColored(0.2f, 0.7f, 1.0f, 1.0f, "[AI] " + config.getProvider().getDisplayName());
            ImGui.sameLine();
            ImGui.textDisabled("(" + config.getActiveModel() + ")");

            ImGui.sameLine(ImGui.getWindowWidth() - 145.0f);
            if (showSettings) {
                ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.6f, 0.9f, 1.0f);
                if (ImGui.smallButton("Settings")) {
                    showSettings = false;
                }
                ImGui.popStyleColor();
            } else {
                if (ImGui.smallButton("Settings")) {
                    showSettings = true;
                }
            }

            ImGui.sameLine();
            if (ImGui.smallButton("Clear")) {
                chatHistory.clear();
                chatHistory.add(new ChatMessage("System", "Chat history cleared.", false));
            }

            ImGui.separator();
            ImGui.spacing();

            if (showSettings) {
                renderSettingsSection();
                ImGui.separator();
                ImGui.spacing();
            }

            float footerHeight = 44.0f;
            ImGui.beginChild("ChatScrollRegion", 0.0f, -footerHeight, true);

            synchronized (chatHistory) {
                for (ChatMessage msg : chatHistory) {
                    if ("User".equals(msg.sender)) {
                        ImGui.textColored(0.2f, 0.85f, 0.4f, 1.0f, "You:");
                        ImGui.sameLine();
                        ImGui.textWrapped(msg.text);
                    } else if ("System".equals(msg.sender)) {
                        ImGui.textColored(0.95f, 0.75f, 0.2f, 1.0f, "[Info] " + msg.text);
                    } else {
                        if (msg.isError) {
                            ImGui.textColored(1.0f, 0.35f, 0.35f, 1.0f, "[Error]");
                            ImGui.sameLine();
                            ImGui.textColored(1.0f, 0.4f, 0.4f, 1.0f, msg.text);
                        } else {
                            ImGui.textColored(0.2f, 0.65f, 1.0f, 1.0f, "AI:");
                            ImGui.sameLine();
                            ImGui.textWrapped(msg.text);
                        }
                    }
                    ImGui.spacing();
                }
            }

            if (isLoading) {
                ImGui.textColored(0.7f, 0.7f, 0.7f, 1.0f, "AI is thinking & planning motion...");
            }

            if (ImGui.getScrollY() >= ImGui.getScrollMaxY()) {
                ImGui.setScrollHereY(1.0f);
            }

            ImGui.endChild();

            ImGui.spacing();

            ImGui.beginDisabled(isLoading);

            ImGui.pushItemWidth(-75.0f);
            boolean enterPressed = ImGui.inputText("##promptInput", promptInput,
                    ImGuiInputTextFlags.EnterReturnsTrue);
            ImGui.popItemWidth();

            ImGui.sameLine();
            boolean sendClicked = ImGui.button(isLoading ? "..." : "Send", 65.0f, 26.0f);

            ImGui.endDisabled();

            if ((enterPressed || sendClicked) && !isLoading) {
                String input = promptInput.get().trim();
                if (!input.isEmpty()) {
                    sendMessage(input);
                    promptInput.set("");
                }
            }
        }
        ImGui.end();

        if (!isOpen.get()) {
            visible = false;
        }
    }

    private void renderSettingsSection() {
        if (ImGui.collapsingHeader("API & Model Settings", ImGuiTreeNodeFlags.DefaultOpen)) {
            ImGui.text("AI Provider:");
            ImGui.sameLine(130.0f);
            if (ImGui.radioButton("Google Gemini", selectedProviderIdx, 0)) {
                config.setProvider(AiConfig.Provider.GEMINI);
                syncModelIndexFromConfig();
                config.save();
            }
            ImGui.sameLine();
            if (ImGui.radioButton("OpenAI", selectedProviderIdx, 1)) {
                config.setProvider(AiConfig.Provider.OPENAI);
                syncModelIndexFromConfig();
                config.save();
            }

            ImGui.spacing();

            if (config.getProvider() == AiConfig.Provider.GEMINI) {
                ImGui.text("Gemini API Key:");
                ImGui.setNextItemWidth(-1.0f);
                int flags = showPassword ? ImGuiInputTextFlags.None : ImGuiInputTextFlags.Password;
                if (ImGui.inputText("##geminiKey", geminiKeyInput, flags)) {
                    config.setGeminiApiKey(geminiKeyInput.get());
                    config.save();
                }

                ImGui.spacing();
                ImGui.text("Model Selection:");
                ImGui.setNextItemWidth(-1.0f);
                if (ImGui.combo("##geminiModel", selectedModelIdx, GEMINI_MODELS)) {
                    config.setGeminiModel(GEMINI_MODELS[selectedModelIdx.get()]);
                    config.save();
                }
                ImGui.textDisabled("Recommended: gemini-3.6-flash");
            } else {
                ImGui.text("OpenAI API Key:");
                ImGui.setNextItemWidth(-1.0f);
                int flags = showPassword ? ImGuiInputTextFlags.None : ImGuiInputTextFlags.Password;
                if (ImGui.inputText("##openaiKey", openAiKeyInput, flags)) {
                    config.setOpenAiApiKey(openAiKeyInput.get());
                    config.save();
                }

                ImGui.spacing();
                ImGui.text("Model Selection:");
                ImGui.setNextItemWidth(-1.0f);
                if (ImGui.combo("##openaiModel", selectedModelIdx, OPENAI_MODELS)) {
                    config.setOpenAiModel(OPENAI_MODELS[selectedModelIdx.get()]);
                    config.save();
                }
                ImGui.textDisabled("Recommended: gpt-4o-mini");
            }

            ImGui.spacing();
            ImBoolean showPassBool = new ImBoolean(showPassword);
            if (ImGui.checkbox("Show API Key", showPassBool)) {
                showPassword = showPassBool.get();
            }

            ImGui.sameLine(ImGui.getWindowWidth() - 110.0f);
            if (ImGui.button("Save", 90.0f, 22.0f)) {
                config.save();
                chatHistory.add(new ChatMessage("System", "Settings saved (" + config.getProvider().getDisplayName()
                        + " - " + config.getActiveModel() + ")", false));
            }
        }
    }

    private void sendMessage(String userPrompt) {
        chatHistory.add(new ChatMessage("User", userPrompt, false));
        isLoading = true;

        String systemContext = buildRobotSystemContext();

        aiService.sendMessageAsync(config, userPrompt, systemContext, new AiService.ResponseCallback() {
            @Override
            public void onSuccess(String responseText) {
                AiActionExecutor.ExecutionResult execRes = AiActionExecutor.processAndExecute(responseText, joints, motionPlayer);
                chatHistory.add(new ChatMessage("AI", execRes.cleanedMessage, false));
                if (execRes.hasActions && engine != null) {
                    engine.autoSaveProject();
                }
                isLoading = false;
            }

            @Override
            public void onError(String errorMessage) {
                chatHistory.add(new ChatMessage("AI", errorMessage, true));
                isLoading = false;
            }
        });
    }

    private String buildRobotSystemContext() {
        StringBuilder sb = new StringBuilder();
        sb.append("You are an intelligent robotics assistant for the 3D Robot Simulator (RSim2).\n");
        sb.append("Answer user questions about the robot, perform kinematic analysis, or generate motion sequences.\n\n");

        if (joints != null && !joints.isEmpty()) {
            sb.append("Currently loaded robot configuration:\n");
            sb.append("- Total Joints: ").append(joints.size()).append("\n");
            sb.append("Joints List:\n");
            for (Joint j : joints) {
                float deg = (float) Math.toDegrees(j.getCurrentAngleRadians());
                float minDeg = (float) Math.toDegrees(j.getMinLimit());
                float maxDeg = (float) Math.toDegrees(j.getMaxLimit());
                sb.append(String.format(
                        "  - ID: %s | Type: %s | Parent: %s | Child: %s | Axis: (%.1f, %.1f, %.1f) | Current Angle: %.1f deg | Limits: [%.0f, %.0f]\n",
                        j.getId(),
                        j.getType().name(),
                        j.getParentNode() != null ? j.getParentNode().getId() : "world",
                        j.getChildNode() != null ? j.getChildNode().getId() : "null",
                        j.getAxis().x, j.getAxis().y, j.getAxis().z,
                        deg, minDeg, maxDeg));
            }

            sb.append("\nTIMED MOTION & TRAJECTORY PROTOCOL (IMPORTANT):\n");
            sb.append("When the user asks you to perform a movement, animation, gesture, or time-series trajectory (e.g. 'wave hand', 'bow', 'raise arm', 'walk step', etc.), you MUST append the following JSON 'animation' block at the very end of your helpful response:\n");
            sb.append("```json\n");
            sb.append("{\n");
            sb.append("  \"animation\": {\n");
            sb.append("    \"name\": \"Motion Name\",\n");
            sb.append("    \"duration\": 2.0,\n");
            sb.append("    \"loop\": true,\n");
            sb.append("    \"keyframes\": [\n");
            sb.append("      {\n");
            sb.append("        \"time\": 0.0,\n");
            sb.append("        \"joints\": {\n");
            sb.append("          \"joint_id_1\": 0.0,\n");
            sb.append("          \"joint_id_2\": 20.0\n");
            sb.append("        }\n");
            sb.append("      },\n");
            sb.append("      {\n");
            sb.append("        \"time\": 0.5,\n");
            sb.append("        \"joints\": {\n");
            sb.append("          \"joint_id_1\": 35.0,\n");
            sb.append("          \"joint_id_2\": 40.0\n");
            sb.append("        }\n");
            sb.append("      },\n");
            sb.append("      {\n");
            sb.append("        \"time\": 1.0,\n");
            sb.append("        \"joints\": {\n");
            sb.append("          \"joint_id_1\": -35.0,\n");
            sb.append("          \"joint_id_2\": 20.0\n");
            sb.append("        }\n");
            sb.append("      },\n");
            sb.append("      {\n");
            sb.append("        \"time\": 2.0,\n");
            sb.append("        \"joints\": {\n");
            sb.append("          \"joint_id_1\": 0.0,\n");
            sb.append("          \"joint_id_2\": 20.0\n");
            sb.append("        }\n");
            sb.append("      }\n");
            sb.append("    ]\n");
            sb.append("  }\n");
            sb.append("}\n");
            sb.append("```\n");
            sb.append("Rules:\n");
            sb.append("1. 'time': Timestamp in seconds starting from 0.0 up to duration.\n");
            sb.append("2. 'joints': Target angle in degrees for each joint at that timestamp. Simulator smoothly interpolates between keyframes.\n");
            sb.append("3. 'loop': Set to true for cyclic motions (waving, walking, idle, etc.).\n");
            sb.append("4. Never exceed joint limits and only use valid joint IDs from the scene.\n");
            sb.append("5. For instantaneous single-step changes, you can use 'actions' (e.g. {\"actions\": [{\"joint\": \"ALL\", \"target_deg\": 0.0}]}).\n");
        } else {
            sb.append("No active robot model is currently loaded in the scene.\n");
        }

        return sb.toString();
    }
}
