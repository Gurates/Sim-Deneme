#version 330 core

in vec3 fragPos3D;
out vec4 outColor;

void main() {
    float gridSize = 1.0;
    
    vec2 coord = fragPos3D.xz;
    vec2 grid = abs(fract(coord / gridSize - 0.5) - 0.5) / fwidth(coord / gridSize);
    float line = min(grid.x, grid.y);
    
    float lineAlpha = 1.0 - min(line, 1.0);
    
    vec4 gridColor = vec4(0.4, 0.4, 0.4, lineAlpha * 0.4);
    
    float fade = max(0.0, 1.0 - (length(fragPos3D) / 30.0));
    
    outColor = gridColor * fade;
    
    if (outColor.a < 0.01) {
        discard;
    }
}
