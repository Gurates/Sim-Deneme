#version 330 core

in vec3 fragNormal;
in vec3 fragPos;

out vec4 outColor;

void main() {
    vec3 norm = normalize(fragNormal);
    vec3 lightDir = normalize(vec3(0.5, 1.0, 0.3));
    
    float diff = max(dot(norm, lightDir), 0.0);
    float ambient = 0.35;
    
    vec3 baseColor = vec3(0.7, 0.75, 0.8);
    vec3 finalColor = baseColor * (ambient + diff * 0.65);
    
    outColor = vec4(finalColor, 1.0);
}
