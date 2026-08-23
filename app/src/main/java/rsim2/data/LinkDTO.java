package rsim2.data;

import java.util.List;

public class LinkDTO {
    public String id;
    public String mesh;
    public List<String> meshes;
    public float mass;
    public float[] scale;
    public float[] rotation;

    public LinkDTO() {}

    public LinkDTO(String id, String mesh, float mass, float[] scale, float[] rotation) {
        this.id = id;
        this.mesh = mesh;
        this.mass = mass;
        this.scale = scale;
        this.rotation = rotation;
    }

    public LinkDTO(String id, List<String> meshes, float mass, float[] scale, float[] rotation) {
        this.id = id;
        this.meshes = meshes;
        this.mass = mass;
        this.scale = scale;
        this.rotation = rotation;
    }
}
