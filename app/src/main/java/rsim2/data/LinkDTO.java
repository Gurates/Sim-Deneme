package rsim2.data;

public class LinkDTO {
    public String id;
    public String mesh;
    public float mass;

    public LinkDTO() {}

    public LinkDTO(String id, String mesh, float mass) {
        this.id = id;
        this.mesh = mesh;
        this.mass = mass;
    }
}
