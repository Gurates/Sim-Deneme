package rsim2.data;

public class LinkDTO {
    public String id;
    public String mesh;
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
}
