package rsim2.data;

import java.util.ArrayList;
import java.util.List;

public class RobotDefinitionDTO {
    public String schemaVersion = "1.0";
    public String robotName;
    public MetadataDTO metadata;
    public List<LinkDTO> links = new ArrayList<>();
    public List<JointDTO> joints = new ArrayList<>();
    public List<Object> sensors = new ArrayList<>();

    public RobotDefinitionDTO() {}
}
