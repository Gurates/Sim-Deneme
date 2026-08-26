package rsim2.ai;

import com.google.gson.*;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class AiService {
    public interface ResponseCallback {
        void onSuccess(String responseText);
        void onError(String errorMessage);
    }

    private final HttpClient httpClient;
    private final ExecutorService executor;
    private final Gson gson;

    public AiService() {
        this.httpClient = HttpClient.newBuilder()
                .connectTimeout(Duration.ofSeconds(15))
                .build();
        this.executor = Executors.newCachedThreadPool(r -> {
            Thread t = new Thread(r, "AI-Service-Thread");
            t.setDaemon(true);
            return t;
        });
        this.gson = new Gson();
    }

    public void sendMessageAsync(AiConfig config, String userPrompt, String systemContext, ResponseCallback callback) {
        if (config == null) {
            callback.onError("AI configuration is missing.");
            return;
        }

        String apiKey = config.getActiveApiKey();
        if (apiKey == null || apiKey.trim().isEmpty()) {
            callback.onError("API Key is missing! Please enter your " + config.getProvider().getDisplayName() + " API key in Settings.");
            return;
        }

        executor.submit(() -> {
            try {
                if (config.getProvider() == AiConfig.Provider.GEMINI) {
                    callGemini(config, userPrompt, systemContext, callback);
                } else {
                    callOpenAi(config, userPrompt, systemContext, callback);
                }
            } catch (Exception e) {
                callback.onError("Error occurred during request: " + e.getMessage());
            }
        });
    }

    private void callGemini(AiConfig config, String userPrompt, String systemContext, ResponseCallback callback) {
        try {
            String model = config.getGeminiModel() != null ? config.getGeminiModel().trim() : "gemini-3.6-flash";
            if (model.startsWith("models/")) {
                model = model.substring("models/".length());
            }
            if (model.isEmpty()) {
                model = "gemini-3.6-flash";
            }
            String apiKey = config.getGeminiApiKey();
            String url = "https://generativelanguage.googleapis.com/v1beta/models/" + model + ":generateContent?key=" + apiKey;

            JsonObject root = new JsonObject();

            if (systemContext != null && !systemContext.trim().isEmpty()) {
                JsonObject sysInst = new JsonObject();
                JsonArray sysParts = new JsonArray();
                JsonObject sysPart = new JsonObject();
                sysPart.addProperty("text", systemContext);
                sysParts.add(sysPart);
                sysInst.add("parts", sysParts);
                root.add("systemInstruction", sysInst);
            }

            JsonArray contents = new JsonArray();
            JsonObject userContent = new JsonObject();
            userContent.addProperty("role", "user");
            JsonArray parts = new JsonArray();
            JsonObject part = new JsonObject();
            part.addProperty("text", userPrompt);
            parts.add(part);
            userContent.add("parts", parts);
            contents.add(userContent);
            root.add("contents", contents);

            String requestJson = gson.toJson(root);

            HttpRequest request = HttpRequest.newBuilder()
                    .uri(URI.create(url))
                    .header("Content-Type", "application/json")
                    .timeout(Duration.ofSeconds(45))
                    .POST(HttpRequest.BodyPublishers.ofString(requestJson))
                    .build();

            HttpResponse<String> response = httpClient.send(request, HttpResponse.BodyHandlers.ofString());

            if (response.statusCode() != 200) {
                String errorMsg = parseErrorMessage(response.body(), response.statusCode());
                callback.onError("Gemini API Error (" + response.statusCode() + "): " + errorMsg);
                return;
            }

            JsonObject resObj = JsonParser.parseString(response.body()).getAsJsonObject();
            if (resObj.has("candidates")) {
                JsonArray candidates = resObj.getAsJsonArray("candidates");
                if (candidates.size() > 0) {
                    JsonObject firstCandidate = candidates.get(0).getAsJsonObject();
                    if (firstCandidate.has("content")) {
                        JsonObject content = firstCandidate.getAsJsonObject("content");
                        if (content.has("parts")) {
                            JsonArray resParts = content.getAsJsonArray("parts");
                            if (resParts.size() > 0) {
                                String reply = resParts.get(0).getAsJsonObject().get("text").getAsString();
                                callback.onSuccess(reply);
                                return;
                            }
                        }
                    }
                }
            }

            callback.onError("Could not parse Gemini API response:\n" + response.body());
        } catch (Exception e) {
            callback.onError("Gemini Connection Error: " + e.getMessage());
        }
    }

    private void callOpenAi(AiConfig config, String userPrompt, String systemContext, ResponseCallback callback) {
        try {
            String model = config.getOpenAiModel();
            String apiKey = config.getOpenAiApiKey();
            String url = "https://api.openai.com/v1/chat/completions";

            JsonObject root = new JsonObject();
            root.addProperty("model", model);

            JsonArray messages = new JsonArray();

            if (systemContext != null && !systemContext.trim().isEmpty()) {
                JsonObject sysMsg = new JsonObject();
                sysMsg.addProperty("role", "system");
                sysMsg.addProperty("content", systemContext);
                messages.add(sysMsg);
            }

            JsonObject userMsg = new JsonObject();
            userMsg.addProperty("role", "user");
            userMsg.addProperty("content", userPrompt);
            messages.add(userMsg);

            root.add("messages", messages);

            String requestJson = gson.toJson(root);

            HttpRequest request = HttpRequest.newBuilder()
                    .uri(URI.create(url))
                    .header("Content-Type", "application/json")
                    .header("Authorization", "Bearer " + apiKey)
                    .timeout(Duration.ofSeconds(45))
                    .POST(HttpRequest.BodyPublishers.ofString(requestJson))
                    .build();

            HttpResponse<String> response = httpClient.send(request, HttpResponse.BodyHandlers.ofString());

            if (response.statusCode() != 200) {
                String errorMsg = parseErrorMessage(response.body(), response.statusCode());
                callback.onError("OpenAI API Error (" + response.statusCode() + "): " + errorMsg);
                return;
            }

            JsonObject resObj = JsonParser.parseString(response.body()).getAsJsonObject();
            if (resObj.has("choices")) {
                JsonArray choices = resObj.getAsJsonArray("choices");
                if (choices.size() > 0) {
                    JsonObject firstChoice = choices.get(0).getAsJsonObject();
                    if (firstChoice.has("message")) {
                        JsonObject message = firstChoice.getAsJsonObject("message");
                        if (message.has("content")) {
                            String reply = message.get("content").getAsString();
                            callback.onSuccess(reply);
                            return;
                        }
                    }
                }
            }

            callback.onError("Could not parse OpenAI API response:\n" + response.body());
        } catch (Exception e) {
            callback.onError("OpenAI Connection Error: " + e.getMessage());
        }
    }

    private String parseErrorMessage(String body, int statusCode) {
        try {
            JsonObject json = JsonParser.parseString(body).getAsJsonObject();
            if (json.has("error")) {
                JsonElement errEl = json.get("error");
                if (errEl.isJsonObject()) {
                    JsonObject errObj = errEl.getAsJsonObject();
                    if (errObj.has("message")) {
                        return errObj.get("message").getAsString();
                    }
                } else if (errEl.isJsonPrimitive()) {
                    return errEl.getAsString();
                }
            }
        } catch (Exception ignored) {
        }
        return "HTTP " + statusCode + " - " + body;
    }
}
