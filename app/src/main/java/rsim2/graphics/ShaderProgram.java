package rsim2.graphics;

import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.Scanner;

import static org.lwjgl.opengl.GL20.*;

public class ShaderProgram {
    private final int programId;
    private int vertexShaderId;
    private int fragmentShaderId;

    public ShaderProgram() throws Exception {
        programId = glCreateProgram();
        if (programId == 0) {
            throw new Exception("Could not create Shader Program");
        }
    }

    public void createVertexShader(String resourcePath) throws Exception {
        vertexShaderId = createShader(resourcePath, GL_VERTEX_SHADER);
    }

    public void createFragmentShader(String resourcePath) throws Exception {
        fragmentShaderId = createShader(resourcePath, GL_FRAGMENT_SHADER);
    }

    private int createShader(String resourcePath, int shaderType) throws Exception {
        String shaderCode = loadResource(resourcePath);
        int shaderId = glCreateShader(shaderType);
        if (shaderId == 0) {
            throw new Exception("Error creating shader. Type: " + shaderType);
        }

        glShaderSource(shaderId, shaderCode);
        glCompileShader(shaderId);

        if (glGetShaderi(shaderId, GL_COMPILE_STATUS) == 0) {
            throw new Exception("Error compiling Shader code (" + resourcePath + "): " + glGetShaderInfoLog(shaderId, 1024));
        }

        glAttachShader(programId, shaderId);
        return shaderId;
    }

    public void link() throws Exception {
        glLinkProgram(programId);
        if (glGetProgrami(programId, GL_LINK_STATUS) == 0) {
            throw new Exception("Error linking Shader code: " + glGetProgramInfoLog(programId, 1024));
        }

        if (vertexShaderId != 0) {
            glDetachShader(programId, vertexShaderId);
            glDeleteShader(vertexShaderId);
        }
        if (fragmentShaderId != 0) {
            glDetachShader(programId, fragmentShaderId);
            glDeleteShader(fragmentShaderId);
        }
    }

    public void bind() {
        glUseProgram(programId);
    }

    public void unbind() {
        glUseProgram(0);
    }

    public int getUniformLocation(String uniformName) {
        return glGetUniformLocation(programId, uniformName);
    }

    public int getProgramId() {
        return programId;
    }

    private String loadResource(String fileName) throws Exception {
        try (InputStream in = getClass().getResourceAsStream(fileName)) {
            if (in == null) throw new Exception("Resource not found: " + fileName);
            Scanner scanner = new Scanner(in, StandardCharsets.UTF_8.name());
            return scanner.useDelimiter("\\A").next();
        }
    }

    public void cleanup() {
        unbind();
        if (programId != 0) {
            glDeleteProgram(programId);
        }
    }
}
