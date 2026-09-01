package rsim2.collision;

import org.joml.Matrix4f;
import org.joml.Vector3f;

public class CollisionMath {

    private static final float EPSILON = 1e-6f;
    private static final int GJK_MAX_ITERATIONS = 32;

    public static boolean intersectShapes(CollisionShape shapeA, Matrix4f transformA,
                                          CollisionShape shapeB, Matrix4f transformB,
                                          float margin) {
        if (shapeA == null || shapeB == null) return false;

        if (shapeA instanceof CompoundShape) {
            CompoundShape compA = (CompoundShape) shapeA;
            for (CompoundShape.Entry childA : compA.getChildren()) {
                Matrix4f childWorldA = childA.computeWorldTransform(transformA);
                if (intersectShapes(childA.getShape(), childWorldA, shapeB, transformB, margin)) {
                    return true;
                }
            }
            return false;
        }

        if (shapeB instanceof CompoundShape) {
            CompoundShape compB = (CompoundShape) shapeB;
            for (CompoundShape.Entry childB : compB.getChildren()) {
                Matrix4f childWorldB = childB.computeWorldTransform(transformB);
                if (intersectShapes(shapeA, transformA, childB.getShape(), childWorldB, margin)) {
                    return true;
                }
            }
            return false;
        }

        if (shapeA instanceof SphereShape && shapeB instanceof SphereShape) {
            return intersectSphereSphere((SphereShape) shapeA, transformA, (SphereShape) shapeB, transformB, margin);
        }
        if (shapeA instanceof CapsuleShape && shapeB instanceof CapsuleShape) {
            return intersectCapsuleCapsule((CapsuleShape) shapeA, transformA, (CapsuleShape) shapeB, transformB, margin);
        }
        if (shapeA instanceof SphereShape && shapeB instanceof CapsuleShape) {
            return intersectSphereCapsule((SphereShape) shapeA, transformA, (CapsuleShape) shapeB, transformB, margin);
        }
        if (shapeA instanceof CapsuleShape && shapeB instanceof SphereShape) {
            return intersectSphereCapsule((SphereShape) shapeB, transformB, (CapsuleShape) shapeA, transformA, margin);
        }

        return intersectGJK(shapeA, transformA, shapeB, transformB, margin);
    }

    public static boolean intersectGJK(CollisionShape shapeA, Matrix4f transformA,
                                       CollisionShape shapeB, Matrix4f transformB,
                                       float margin) {
        Vector3f[] simplex = new Vector3f[4];
        simplex[0] = new Vector3f();
        simplex[1] = new Vector3f();
        simplex[2] = new Vector3f();
        simplex[3] = new Vector3f();
        int simplexSize = 0;

        Vector3f dir = new Vector3f(1.0f, 0.0f, 0.0f);
        Vector3f centerA = new Vector3f();
        Vector3f centerB = new Vector3f();
        if (transformA != null) transformA.transformPosition(0, 0, 0, centerA);
        if (transformB != null) transformB.transformPosition(0, 0, 0, centerB);
        centerB.sub(centerA, dir);
        if (dir.lengthSquared() < EPSILON) {
            dir.set(1.0f, 0.0f, 0.0f);
        }

        minkowskiSupport(shapeA, transformA, shapeB, transformB, dir, simplex[0]);
        simplexSize = 1;
        dir.set(simplex[0]).negate();

        for (int iter = 0; iter < GJK_MAX_ITERATIONS; iter++) {
            if (dir.lengthSquared() < EPSILON) {
                return true;
            }

            Vector3f a = new Vector3f();
            minkowskiSupport(shapeA, transformA, shapeB, transformB, dir, a);

            if (a.dot(dir) < -margin) {
                return false;
            }

            for (int k = simplexSize; k > 0; k--) {
                if (k < 4) simplex[k].set(simplex[k - 1]);
            }
            simplex[0].set(a);
            simplexSize++;

            if (doSimplex(simplex, simplexSize, dir)) {
                return true;
            }

            simplexSize = nextSimplexSize;
        }

        return false;
    }

    private static int nextSimplexSize = 0;

    private static void minkowskiSupport(CollisionShape shapeA, Matrix4f transformA,
                                         CollisionShape shapeB, Matrix4f transformB,
                                         Vector3f dir, Vector3f out) {
        Vector3f supA = new Vector3f();
        Vector3f supB = new Vector3f();
        Vector3f negDir = new Vector3f(dir).negate();

        shapeA.getSupportPoint(dir, transformA, supA);
        shapeB.getSupportPoint(negDir, transformB, supB);

        supA.sub(supB, out);
    }

    private static boolean doSimplex(Vector3f[] s, int size, Vector3f dir) {
        Vector3f a = s[0];

        if (size == 2) {
            Vector3f b = s[1];
            Vector3f ab = new Vector3f(b).sub(a);
            Vector3f ao = new Vector3f(a).negate();

            if (ab.dot(ao) > 0) {
                Vector3f abCrossAo = new Vector3f();
                ab.cross(ao, abCrossAo);
                abCrossAo.cross(ab, dir);
                nextSimplexSize = 2;
            } else {
                dir.set(ao);
                s[0].set(a);
                nextSimplexSize = 1;
            }
            return false;
        } else if (size == 3) {
            Vector3f b = s[1];
            Vector3f c = s[2];
            Vector3f ab = new Vector3f(b).sub(a);
            Vector3f ac = new Vector3f(c).sub(a);
            Vector3f ao = new Vector3f(a).negate();

            Vector3f abc = new Vector3f();
            ab.cross(ac, abc);

            Vector3f abNorm = new Vector3f();
            ab.cross(abc, abNorm);

            Vector3f acNorm = new Vector3f();
            abc.cross(ac, acNorm);

            if (abNorm.dot(ao) > 0) {
                s[0].set(a);
                s[1].set(b);
                nextSimplexSize = 2;
                abNorm.cross(ab, dir);
                if (dir.lengthSquared() < EPSILON) dir.set(ao);
                return false;
            } else if (acNorm.dot(ao) > 0) {
                s[0].set(a);
                s[1].set(c);
                nextSimplexSize = 2;
                acNorm.cross(ac, dir);
                if (dir.lengthSquared() < EPSILON) dir.set(ao);
                return false;
            } else {
                if (abc.dot(ao) > 0) {
                    dir.set(abc);
                    nextSimplexSize = 3;
                } else {
                    dir.set(abc).negate();
                    s[0].set(a);
                    s[1].set(c);
                    s[2].set(b);
                    nextSimplexSize = 3;
                }
                return false;
            }
        } else if (size == 4) {
            Vector3f b = s[1];
            Vector3f c = s[2];
            Vector3f d = s[3];

            Vector3f ab = new Vector3f(b).sub(a);
            Vector3f ac = new Vector3f(c).sub(a);
            Vector3f ad = new Vector3f(d).sub(a);
            Vector3f ao = new Vector3f(a).negate();

            Vector3f abc = new Vector3f();
            ab.cross(ac, abc);
            Vector3f acd = new Vector3f();
            ac.cross(ad, acd);
            Vector3f adb = new Vector3f();
            ad.cross(ab, adb);

            if (abc.dot(ao) > 0) {
                s[0].set(a);
                s[1].set(b);
                s[2].set(c);
                dir.set(abc);
                nextSimplexSize = 3;
                return false;
            }
            if (acd.dot(ao) > 0) {
                s[0].set(a);
                s[1].set(c);
                s[2].set(d);
                dir.set(acd);
                nextSimplexSize = 3;
                return false;
            }
            if (adb.dot(ao) > 0) {
                s[0].set(a);
                s[1].set(d);
                s[2].set(b);
                dir.set(adb);
                nextSimplexSize = 3;
                return false;
            }

            return true;
        }

        nextSimplexSize = 1;
        dir.set(-a.x, -a.y, -a.z);
        return false;
    }

    public static boolean intersectSphereSphere(SphereShape a, Matrix4f tA,
                                                SphereShape b, Matrix4f tB,
                                                float margin) {
        Vector3f cA = new Vector3f();
        Vector3f cB = new Vector3f();
        if (tA != null) tA.transformPosition(a.getLocalCenter(), cA);
        else cA.set(a.getLocalCenter());
        if (tB != null) tB.transformPosition(b.getLocalCenter(), cB);
        else cB.set(b.getLocalCenter());

        float distSq = cA.distanceSquared(cB);
        float radiusSum = a.getRadius() + b.getRadius() + margin;
        return distSq <= (radiusSum * radiusSum);
    }

    public static boolean intersectCapsuleCapsule(CapsuleShape a, Matrix4f tA,
                                                  CapsuleShape b, Matrix4f tB,
                                                  float margin) {
        Vector3f a0 = new Vector3f();
        Vector3f a1 = new Vector3f();
        Vector3f b0 = new Vector3f();
        Vector3f b1 = new Vector3f();

        if (tA != null) {
            tA.transformPosition(a.getLocalP0(), a0);
            tA.transformPosition(a.getLocalP1(), a1);
        } else {
            a0.set(a.getLocalP0());
            a1.set(a.getLocalP1());
        }

        if (tB != null) {
            tB.transformPosition(b.getLocalP0(), b0);
            tB.transformPosition(b.getLocalP1(), b1);
        } else {
            b0.set(b.getLocalP0());
            b1.set(b.getLocalP1());
        }

        float distSq = segmentSegmentDistanceSquared(a0, a1, b0, b1);
        float radiusSum = a.getRadius() + b.getRadius() + margin;
        return distSq <= (radiusSum * radiusSum);
    }

    public static boolean intersectSphereCapsule(SphereShape s, Matrix4f tS,
                                                 CapsuleShape c, Matrix4f tC,
                                                 float margin) {
        Vector3f sC = new Vector3f();
        if (tS != null) tS.transformPosition(s.getLocalCenter(), sC);
        else sC.set(s.getLocalCenter());

        Vector3f c0 = new Vector3f();
        Vector3f c1 = new Vector3f();
        if (tC != null) {
            tC.transformPosition(c.getLocalP0(), c0);
            tC.transformPosition(c.getLocalP1(), c1);
        } else {
            c0.set(c.getLocalP0());
            c1.set(c.getLocalP1());
        }

        float distSq = pointSegmentDistanceSquared(sC, c0, c1);
        float radiusSum = s.getRadius() + c.getRadius() + margin;
        return distSq <= (radiusSum * radiusSum);
    }

    public static float pointSegmentDistanceSquared(Vector3f p, Vector3f a, Vector3f b) {
        Vector3f ab = new Vector3f(b).sub(a);
        Vector3f ap = new Vector3f(p).sub(a);

        float abLenSq = ab.lengthSquared();
        if (abLenSq < EPSILON) {
            return ap.lengthSquared();
        }

        float t = ap.dot(ab) / abLenSq;
        t = Math.max(0.0f, Math.min(1.0f, t));

        Vector3f closest = new Vector3f(a).add(ab.mul(t));
        return p.distanceSquared(closest);
    }

    public static float segmentSegmentDistanceSquared(Vector3f p1, Vector3f q1,
                                                      Vector3f p2, Vector3f q2) {
        Vector3f d1 = new Vector3f(q1).sub(p1);
        Vector3f d2 = new Vector3f(q2).sub(p2);
        Vector3f r = new Vector3f(p1).sub(p2);

        float a = d1.dot(d1);
        float e = d2.dot(d2);
        float f = d2.dot(r);

        if (a <= EPSILON && e <= EPSILON) {
            return r.lengthSquared();
        }

        float s, t;
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
}
