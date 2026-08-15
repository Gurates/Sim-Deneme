package rsim2.scene;

public enum JointType {
    REVOLUTE,
    FIXED;

    public static JointType fromString(String type) {
        if (type == null) {
            return REVOLUTE;
        }
        try {
            return JointType.valueOf(type.trim().toUpperCase());
        } catch (IllegalArgumentException e) {
            return REVOLUTE;
        }
    }
}
