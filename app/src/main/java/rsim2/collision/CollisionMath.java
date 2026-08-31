package rsim2.collision;

import org.joml.Vector3f;

public class CollisionMath {

    private static final float EPSILON = 1e-6f;

    public static boolean intersectOBBOBB(OBB a, OBB b) {
        if (a == null || b == null) return false;

        Vector3f cA = a.getCenter();
        Vector3f cB = b.getCenter();
        Vector3f[] uA = a.getAxes();
        Vector3f[] uB = b.getAxes();
        Vector3f eA = a.getHalfExtents();
        Vector3f eB = b.getHalfExtents();

        Vector3f v = new Vector3f(cB).sub(cA);

        float tA0 = v.dot(uA[0]);
        float tA1 = v.dot(uA[1]);
        float tA2 = v.dot(uA[2]);

        float[][] R = new float[3][3];
        float[][] absR = new float[3][3];

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R[i][j] = uA[i].dot(uB[j]);
                absR[i][j] = Math.abs(R[i][j]) + EPSILON;
            }
        }

        float ra = eA.x;
        float rb = eB.x * absR[0][0] + eB.y * absR[0][1] + eB.z * absR[0][2];
        if (Math.abs(tA0) > ra + rb) return false;

        ra = eA.y;
        rb = eB.x * absR[1][0] + eB.y * absR[1][1] + eB.z * absR[1][2];
        if (Math.abs(tA1) > ra + rb) return false;

        ra = eA.z;
        rb = eB.x * absR[2][0] + eB.y * absR[2][1] + eB.z * absR[2][2];
        if (Math.abs(tA2) > ra + rb) return false;

        ra = eA.x * absR[0][0] + eA.y * absR[1][0] + eA.z * absR[2][0];
        rb = eB.x;
        if (Math.abs(tA0 * R[0][0] + tA1 * R[1][0] + tA2 * R[2][0]) > ra + rb) return false;

        ra = eA.x * absR[0][1] + eA.y * absR[1][1] + eA.z * absR[2][1];
        rb = eB.y;
        if (Math.abs(tA0 * R[0][1] + tA1 * R[1][1] + tA2 * R[2][1]) > ra + rb) return false;

        ra = eA.x * absR[0][2] + eA.y * absR[1][2] + eA.z * absR[2][2];
        rb = eB.z;
        if (Math.abs(tA0 * R[0][2] + tA1 * R[1][2] + tA2 * R[2][2]) > ra + rb) return false;

        ra = eA.y * absR[2][0] + eA.z * absR[1][0];
        rb = eB.y * absR[0][2] + eB.z * absR[0][1];
        if (Math.abs(tA2 * R[1][0] - tA1 * R[2][0]) > ra + rb) return false;

        ra = eA.y * absR[2][1] + eA.z * absR[1][1];
        rb = eB.x * absR[0][2] + eB.z * absR[0][0];
        if (Math.abs(tA2 * R[1][1] - tA1 * R[2][1]) > ra + rb) return false;

        ra = eA.y * absR[2][2] + eA.z * absR[1][2];
        rb = eB.x * absR[0][1] + eB.y * absR[0][0];
        if (Math.abs(tA2 * R[1][2] - tA1 * R[2][2]) > ra + rb) return false;

        ra = eA.x * absR[2][0] + eA.z * absR[0][0];
        rb = eB.y * absR[1][2] + eB.z * absR[1][1];
        if (Math.abs(tA0 * R[2][0] - tA2 * R[0][0]) > ra + rb) return false;

        ra = eA.x * absR[2][1] + eA.z * absR[0][1];
        rb = eB.x * absR[1][2] + eB.z * absR[1][0];
        if (Math.abs(tA0 * R[2][1] - tA2 * R[0][1]) > ra + rb) return false;

        ra = eA.x * absR[2][2] + eA.z * absR[0][2];
        rb = eB.x * absR[1][1] + eB.y * absR[1][0];
        if (Math.abs(tA0 * R[2][2] - tA2 * R[0][2]) > ra + rb) return false;

        ra = eA.x * absR[1][0] + eA.y * absR[0][0];
        rb = eB.y * absR[2][2] + eB.z * absR[2][1];
        if (Math.abs(tA1 * R[0][0] - tA0 * R[1][0]) > ra + rb) return false;

        ra = eA.x * absR[1][1] + eA.y * absR[0][1];
        rb = eB.x * absR[2][2] + eB.z * absR[2][0];
        if (Math.abs(tA1 * R[0][1] - tA0 * R[1][1]) > ra + rb) return false;

        ra = eA.x * absR[1][2] + eA.y * absR[0][2];
        rb = eB.x * absR[2][1] + eB.y * absR[2][0];
        if (Math.abs(tA1 * R[0][2] - tA0 * R[1][2]) > ra + rb) return false;

        return true;
    }

    public static boolean intersectSphereSphere(Vector3f centerA, float radiusA, Vector3f centerB, float radiusB) {
        float distSq = centerA.distanceSquared(centerB);
        float radiusSum = radiusA + radiusB;
        return distSq <= (radiusSum * radiusSum);
    }

    public static boolean intersectCapsuleCapsule(Vector3f p1, Vector3f q1, float r1,
                                                 Vector3f p2, Vector3f q2, float r2) {
        float distSq = segmentSegmentDistSq(p1, q1, p2, q2);
        float radiusSum = r1 + r2;
        return distSq <= (radiusSum * radiusSum);
    }

    public static float segmentSegmentDistSq(Vector3f p1, Vector3f q1, Vector3f p2, Vector3f q2) {
        Vector3f d1 = new Vector3f(q1).sub(p1);
        Vector3f d2 = new Vector3f(q2).sub(p2);
        Vector3f r = new Vector3f(p1).sub(p2);
        float a = d1.dot(d1);
        float e = d2.dot(d2);
        float f = d2.dot(r);

        float s, t;

        if (a <= EPSILON && e <= EPSILON) {
            return r.dot(r);
        }
        if (a <= EPSILON) {
            s = 0.0f;
            t = Math.max(0.0f, Math.min(1.0f, f / e));
        } else {
            float c = d1.dot(r);
            if (e <= EPSILON) {
                t = 0.0f;
                s = Math.max(0.0f, Math.min(1.0f, -c / a));
            } else {
                float b = d1.dot(d2);
                float denom = a * e - b * b;

                if (denom != 0.0f) {
                    s = Math.max(0.0f, Math.min(1.0f, (b * f - c * e) / denom));
                } else {
                    s = 0.0f;
                }

                t = (b * s + f) / e;

                if (t < 0.0f) {
                    t = 0.0f;
                    s = Math.max(0.0f, Math.min(1.0f, -c / a));
                } else if (t > 1.0f) {
                    t = 1.0f;
                    s = Math.max(0.0f, Math.min(1.0f, (b - c) / a));
                }
            }
        }

        Vector3f c1 = new Vector3f(p1).add(new Vector3f(d1).mul(s));
        Vector3f c2 = new Vector3f(p2).add(new Vector3f(d2).mul(t));
        return c1.distanceSquared(c2);
    }
}
