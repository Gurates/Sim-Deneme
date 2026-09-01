package rsim2.collision;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.joml.Vector4f;

public class AABB {
    private final Vector3f min = new Vector3f(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
    private final Vector3f max = new Vector3f(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE);

    public AABB() {
    }

    public AABB(Vector3f min, Vector3f max) {
        if (min != null) this.min.set(min);
        if (max != null) this.max.set(max);
    }

    public AABB(float minX, float minY, float minZ, float maxX, float maxY, float maxZ) {
        this.min.set(minX, minY, minZ);
        this.max.set(maxX, maxY, maxZ);
    }

    public void set(Vector3f min, Vector3f max) {
        if (min != null) this.min.set(min);
        if (max != null) this.max.set(max);
    }

    public void set(AABB other) {
        if (other != null) {
            this.min.set(other.min);
            this.max.set(other.max);
        }
    }

    public void reset() {
        this.min.set(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
        this.max.set(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE);
    }

    public void clear() {
        reset();
    }

    public Vector3f getMin() {
        return min;
    }

    public Vector3f getMax() {
        return max;
    }

    public Vector3f getCenter() {
        return new Vector3f(min).add(max).mul(0.5f);
    }

    public Vector3f getCenter(Vector3f dest) {
        if (dest == null) dest = new Vector3f();
        return dest.set(min).add(max).mul(0.5f);
    }

    public Vector3f getExtents() {
        return new Vector3f(max).sub(min);
    }

    public Vector3f getHalfExtents() {
        return new Vector3f(max).sub(min).mul(0.5f);
    }

    public Vector3f getHalfExtents(Vector3f dest) {
        if (dest == null) dest = new Vector3f();
        return dest.set(max).sub(min).mul(0.5f);
    }

    public boolean isValid() {
        return min.x <= max.x && min.y <= max.y && min.z <= max.z;
    }

    public void expandBy(Vector3f point) {
        if (point == null) return;
        min.min(point);
        max.max(point);
    }

    public void union(AABB other) {
        if (other == null || !other.isValid()) return;
        min.min(other.min);
        max.max(other.max);
    }

    public boolean intersects(AABB other) {
        if (other == null || !this.isValid() || !other.isValid()) {
            return false;
        }
        return (this.min.x <= other.max.x && this.max.x >= other.min.x) &&
               (this.min.y <= other.max.y && this.max.y >= other.min.y) &&
               (this.min.z <= other.max.z && this.max.z >= other.min.z);
    }

    public boolean contains(Vector3f point) {
        if (point == null || !isValid()) return false;
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y &&
               point.z >= min.z && point.z <= max.z;
    }

    public void transform(Matrix4f transform, AABB outWorldAABB) {
        if (transform == null || outWorldAABB == null || !isValid()) return;

        outWorldAABB.reset();

        Vector4f v = new Vector4f();
        float[] xs = {min.x, max.x};
        float[] ys = {min.y, max.y};
        float[] zs = {min.z, max.z};

        for (float x : xs) {
            for (float y : ys) {
                for (float z : zs) {
                    v.set(x, y, z, 1.0f);
                    transform.transform(v);
                    outWorldAABB.expandBy(new Vector3f(v.x, v.y, v.z));
                }
            }
        }
    }

    @Override
    public String toString() {
        return String.format("AABB[min=(%.3f, %.3f, %.3f), max=(%.3f, %.3f, %.3f)]",
                min.x, min.y, min.z, max.x, max.y, max.z);
    }
}
