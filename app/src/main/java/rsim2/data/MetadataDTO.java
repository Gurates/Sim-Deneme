package rsim2.data;

public class MetadataDTO {
    public String author;
    public String description;
    public Float metersPerSceneUnit;

    public MetadataDTO() {}

    public MetadataDTO(String author, String description) {
        this.author = author;
        this.description = description;
    }
}
