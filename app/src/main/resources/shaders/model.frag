#version 330 core

in vec3 fragNormal;
in vec3 fragPos;

uniform int isSelected;
uniform int isColliding;
uniform vec3 cameraPos;

out vec4 outColor;

void main() {
    vec3 norm = normalize(fragNormal);
    vec3 lightDir = normalize(vec3(0.5, 1.0, 0.4));
    
    float diff = max(dot(norm, lightDir), 0.0);
    float ambient = 0.35;
    
    vec3 baseColor = vec3(0.72, 0.76, 0.82);
    
    if (isColliding == 1) {
        baseColor = vec3(0.95, 0.24, 0.22);
        
        vec3 viewDir = normalize(cameraPos - fragPos);
        float rim = 1.0 - max(dot(viewDir, norm), 0.0);
        rim = pow(rim, 2.0);
        
        vec3 rimColor = vec3(1.0, 0.6, 0.3);
        vec3 litColor = baseColor * (ambient + diff * 0.65);
        vec3 finalColor = mix(litColor, rimColor, rim * 0.9);
        
        outColor = vec4(finalColor, 1.0);
    } else if (isSelected == 1) {
        baseColor = vec3(0.88, 0.82, 0.72);
        
        vec3 viewDir = normalize(cameraPos - fragPos);
        float rim = 1.0 - max(dot(viewDir, norm), 0.0);
        rim = pow(rim, 2.2);
        
        vec3 rimColor = vec3(0.15, 0.85, 1.0);
        vec3 litColor = baseColor * (ambient + diff * 0.65);
        vec3 finalColor = mix(litColor, rimColor, rim * 0.85);
        
        outColor = vec4(finalColor, 1.0);
    } else {
        vec3 finalColor = baseColor * (ambient + diff * 0.65);
        outColor = vec4(finalColor, 1.0);
    }
}
