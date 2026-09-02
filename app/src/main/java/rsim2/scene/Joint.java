package rsim2.scene;

import org.joml.Vector3f;

public class Joint {
    private String id;
    private JointType type;
    private SceneNode parentNode;
    private SceneNode childNode;
    private Vector3f axis;
    private float currentAngleRadians;
    private float minLimit;
    private float maxLimit;
    private float maxSpeed;
    private MotorController motor;

    private int pin = -1;
    private boolean inverted = false;
    private float zeroOffsetDeg = 0.0f;
    private String motorType = "SERVO_PWM";

    public Joint(String id, SceneNode parentNode, SceneNode childNode, Vector3f axis) {
        this(id, parentNode, childNode, JointType.REVOLUTE, axis, -(float) Math.PI, (float) Math.PI, 2.0f);
    }

    public Joint(String id, SceneNode parentNode, SceneNode childNode, JointType type, Vector3f axis) {
        this(id, parentNode, childNode, type, axis, -(float) Math.PI, (float) Math.PI, 2.0f);
    }

    public Joint(String id, SceneNode parentNode, SceneNode childNode, JointType type, Vector3f axis, float minLimit, float maxLimit, float maxSpeed) {
        this.id = id;
        this.type = type != null ? type : JointType.REVOLUTE;
        this.parentNode = parentNode;
        this.childNode = childNode;
        this.currentAngleRadians = 0.0f;
        this.minLimit = minLimit;
        this.maxLimit = maxLimit;
        this.maxSpeed = Math.max(0.001f, maxSpeed);

        if (this.type == JointType.FIXED) {
            this.axis = axis != null ? new Vector3f(axis) : new Vector3f(0.0f, 0.0f, 0.0f);
            this.motor = null;
        } else {
            this.axis = (axis != null && axis.lengthSquared() > 0.0001f) ? new Vector3f(axis).normalize() : new Vector3f(0.0f, 1.0f, 0.0f);
            this.motor = new MotorController(this, this.maxSpeed, this.minLimit, this.maxLimit);
        }
    }

    public void setAngle(float radians) {
        if (type == JointType.FIXED) {
            return;
        }
        this.currentAngleRadians = radians;
        if (childNode != null) {
            childNode.getLocalRotation().fromAxisAngleRad(axis, radians);
        }
    }

    public void setLimits(float minLimit, float maxLimit, float maxSpeed) {
        this.minLimit = minLimit;
        this.maxLimit = maxLimit;
        this.maxSpeed = Math.max(0.001f, maxSpeed);
        if (this.motor != null) {
            this.motor.setMinLimitRadians(minLimit);
            this.motor.setMaxLimitRadians(maxLimit);
            this.motor.setMaxSpeedRadiansPerSecond(this.maxSpeed);
        }
    }

    public String getId() {
        return id;
    }

    public JointType getType() {
        return type;
    }

    public void setType(JointType type) {
        this.type = type;
        if (type == JointType.FIXED) {
            this.motor = null;
        } else if (this.motor == null) {
            this.motor = new MotorController(this, this.maxSpeed, this.minLimit, this.maxLimit);
        }
    }

    public SceneNode getParentNode() {
        return parentNode;
    }

    public SceneNode getChildNode() {
        return childNode;
    }

    public Vector3f getAxis() {
        return axis;
    }

    public float getCurrentAngleRadians() {
        return currentAngleRadians;
    }

    public float getMinLimit() {
        return minLimit;
    }

    public float getMaxLimit() {
        return maxLimit;
    }

    public float getMaxSpeed() {
        return maxSpeed;
    }

    public MotorController getMotor() {
        return motor;
    }

    public void setMotor(MotorController motor) {
        this.motor = motor;
    }

    public int getPin() {
        return pin;
    }

    public void setPin(int pin) {
        this.pin = pin;
    }

    public boolean isInverted() {
        return inverted;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public float getZeroOffsetDeg() {
        return zeroOffsetDeg;
    }

    public void setZeroOffsetDeg(float zeroOffsetDeg) {
        this.zeroOffsetDeg = zeroOffsetDeg;
    }

    public String getMotorType() {
        return motorType;
    }

    public void setMotorType(String motorType) {
        this.motorType = motorType != null ? motorType : "SERVO_PWM";
    }

    public float getAcceleration() {
        return motor != null ? motor.getAcceleration() : 8.0f;
    }

    public void setAcceleration(float acceleration) {
        if (motor != null) {
            motor.setAcceleration(acceleration);
        }
    }

    public float getTargetSpeedRatio() {
        return motor != null ? motor.getTargetSpeedRatio() : 1.0f;
    }

    public void setTargetSpeedRatio(float targetSpeedRatio) {
        if (motor != null) {
            motor.setTargetSpeedRatio(targetSpeedRatio);
        }
    }
}

