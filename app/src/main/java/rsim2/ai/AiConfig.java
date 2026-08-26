package rsim2.ai;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

public class AiConfig {
    public enum Provider {
        GEMINI("Google Gemini"),
        OPENAI("OpenAI");

        private final String displayName;

        Provider(String displayName) {
            this.displayName = displayName;
        }

        public String getDisplayName() {
            return displayName;
        }
    }

    private Provider provider = Provider.GEMINI;
    private String geminiApiKey = "";
    private String geminiModel = "gemini-3.6-flash";
    private String openAiApiKey = "";
    private String openAiModel = "gpt-4o-mini";

    private static final String CONFIG_FILE_NAME = "ai_config.json";
    private static final Gson GSON = new GsonBuilder().setPrettyPrinting().create();

    public static AiConfig load() {
        File file = new File(CONFIG_FILE_NAME);
        if (file.exists() && file.isFile()) {
            try (FileReader reader = new FileReader(file)) {
                AiConfig config = GSON.fromJson(reader, AiConfig.class);
                if (config != null) {
                    return config;
                }
            } catch (Exception e) {
                System.err.println("Failed to load AI config: " + e.getMessage());
            }
        }
        return new AiConfig();
    }

    public void save() {
        try (FileWriter writer = new FileWriter(CONFIG_FILE_NAME)) {
            GSON.toJson(this, writer);
        } catch (Exception e) {
            System.err.println("Failed to save AI config: " + e.getMessage());
        }
    }

    public Provider getProvider() {
        return provider;
    }

    public void setProvider(Provider provider) {
        this.provider = provider != null ? provider : Provider.GEMINI;
    }

    public String getGeminiApiKey() {
        return geminiApiKey;
    }

    public void setGeminiApiKey(String geminiApiKey) {
        this.geminiApiKey = geminiApiKey != null ? geminiApiKey.trim() : "";
    }

    public String getGeminiModel() {
        return (geminiModel != null && !geminiModel.isEmpty()) ? geminiModel : "gemini-3.6-flash";
    }

    public void setGeminiModel(String geminiModel) {
        this.geminiModel = (geminiModel != null && !geminiModel.trim().isEmpty()) ? geminiModel.trim() : "gemini-3.6-flash";
    }

    public String getOpenAiApiKey() {
        return openAiApiKey;
    }

    public void setOpenAiApiKey(String openAiApiKey) {
        this.openAiApiKey = openAiApiKey != null ? openAiApiKey.trim() : "";
    }

    public String getOpenAiModel() {
        return (openAiModel != null && !openAiModel.isEmpty()) ? openAiModel : "gpt-4o-mini";
    }

    public void setOpenAiModel(String openAiModel) {
        this.openAiModel = (openAiModel != null && !openAiModel.trim().isEmpty()) ? openAiModel.trim() : "gpt-4o-mini";
    }

    public String getActiveApiKey() {
        return (provider == Provider.GEMINI) ? geminiApiKey : openAiApiKey;
    }

    public String getActiveModel() {
        return (provider == Provider.GEMINI) ? getGeminiModel() : getOpenAiModel();
    }
}
