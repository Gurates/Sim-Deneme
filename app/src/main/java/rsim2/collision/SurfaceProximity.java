package rsim2.collision;

import org.joml.Intersectiond;
import org.joml.Vector3d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Closest witnesses on world-space triangles; solid overlap is assessed only for closed surfaces. */
public final class SurfaceProximity {
    private SurfaceProximity() {}

    public record Triangle(int index, Vector3d a, Vector3d b, Vector3d c, Vector3d normal, Vector3d center) {
        public Triangle(int index, Vector3d a, Vector3d b, Vector3d c) {
            this(index, a, b, c, b.sub(a, new Vector3d()).cross(c.sub(a, new Vector3d())).normalize(),
                    new Vector3d(a).add(b).add(c).div(3));
        }

        Vector3d vertex(int i) { return switch (i) { case 0 -> a; case 1 -> b; default -> c; }; }
    }

    public record Measurement(String state, double surfaceDistance, double tolerance, Triangle jawSurface,
                              Triangle objectSurface, Vector3d jawPoint, Vector3d objectPoint) {}

    private record Edge(Vector3d low, Vector3d high) {
        static Edge between(Vector3d a, Vector3d b) {
            int cmp = Double.compare(a.x, b.x);
            if (cmp == 0) cmp = Double.compare(a.y, b.y);
            if (cmp == 0) cmp = Double.compare(a.z, b.z);
            return cmp <= 0 ? new Edge(a,b) : new Edge(b,a);
        }
    }

    /** Reconstruct the boundary of a convex support-point cloud without inventing mesh connectivity. */
    public static List<Triangle> convexHull(List<Vector3d> points, Runnable checkActive) {
        if (points.size() < 4) throw new IllegalArgumentException("A solid convex collider needs four noncoplanar points.");
        Vector3d a = points.get(0);
        Vector3d b = points.stream().max(Comparator.comparingDouble(a::distanceSquared)).orElseThrow();
        Vector3d axis = b.sub(a, new Vector3d());
        Vector3d c = points.stream().max(Comparator.comparingDouble(p ->
                axis.cross(p.sub(a, new Vector3d()), new Vector3d()).lengthSquared())).orElseThrow();
        Triangle base = new Triangle(0, a, b, c);
        Vector3d d = points.stream().max(Comparator.comparingDouble(p -> Math.abs(planeDistance(p, base)))).orElseThrow();
        double eps = axis.length() * 1e-10;
        if (!base.normal.isFinite() || Math.abs(planeDistance(d, base)) <= eps)
            throw new IllegalArgumentException("Convex collider has no solid volume. Select an indexed mesh surface.");
        Vector3d inside = new Vector3d(a).add(b).add(c).add(d).div(4);
        List<Triangle> hull = new ArrayList<>(List.of(outward(a,b,c,inside), outward(a,b,d,inside),
                outward(a,c,d,inside), outward(b,c,d,inside)));
        // Each edge shared by two visible faces cancels, leaving the horizon around the new point.
        for (Vector3d p : points) {
            checkActive.run();
            Map<Edge, Integer> edges = new java.util.LinkedHashMap<>();
            for (var it = hull.iterator(); it.hasNext();) {
                Triangle face = it.next();
                if (planeDistance(p, face) <= eps) continue;
                it.remove();
                for (int i = 0; i < 3; i++) edges.merge(Edge.between(face.vertex(i), face.vertex((i+1)%3)), 1, Integer::sum);
            }
            for (var edge : edges.entrySet()) {
                if (edge.getValue() == 1) hull.add(outward(edge.getKey().low, edge.getKey().high, p, inside));
            }
        }
        List<Triangle> result = new ArrayList<>(hull.size());
        for (Triangle face : hull) result.add(new Triangle(result.size(), face.a, face.b, face.c));
        return result;
    }

    private static Triangle outward(Vector3d a, Vector3d b, Vector3d c, Vector3d inside) {
        Triangle t = new Triangle(0,a,b,c);
        return planeDistance(inside,t) > 0 ? new Triangle(0,a,c,b) : t;
    }

    public static final class Surface {
        private final Triangle[] faces;
        private final Branch root;
        private final boolean closed;

        public Surface(List<Triangle> triangles, Runnable checkActive) {
            faces = triangles.stream().filter(t -> t.normal.isFinite()).toArray(Triangle[]::new);
            if (faces.length == 0) throw new IllegalArgumentException("Surface has no nondegenerate triangles.");
            // Welding by position also handles STL files with independent vertices per facet.
            Map<Edge, Integer> edges = new HashMap<>();
            for (int n = 0; n < faces.length; n++) {
                if ((n & 255) == 0) checkActive.run();
                Triangle t = faces[n];
                for (int i = 0; i < 3; i++) edges.merge(Edge.between(t.vertex(i), t.vertex((i+1)%3)), 1, Integer::sum);
            }
            closed = faces.length >= 4 && edges.values().stream().allMatch(count -> count == 2);
            root = new Branch(faces, 0, faces.length, checkActive);
        }
        public boolean closed() { return closed; }

        List<Triangle> triangles() { return java.util.Collections.unmodifiableList(Arrays.asList(faces)); }
    }

    private static class Bounds {
        final Vector3d min = new Vector3d(Double.POSITIVE_INFINITY), max = new Vector3d(Double.NEGATIVE_INFINITY);
        void include(Triangle t) { min.min(t.a).min(t.b).min(t.c); max.max(t.a).max(t.b).max(t.c); }
        double size() { return min.distance(max); }
        double distanceSquared(Bounds b) {
            double x = Math.max(0, Math.max(min.x - b.max.x, b.min.x - max.x));
            double y = Math.max(0, Math.max(min.y - b.max.y, b.min.y - max.y));
            double z = Math.max(0, Math.max(min.z - b.max.z, b.min.z - max.z));
            return x*x + y*y + z*z;
        }
    }

    private static final class Branch extends Bounds {
        final int from, to;
        final Branch left, right;
        Branch(Triangle[] faces, int from, int to, Runnable checkActive) {
            this.from = from;
            this.to = to;
            checkActive.run();
            for (int i=from; i<to; i++) include(faces[i]);
            if (to - from <= 8) { left = right = null; return; }
            int axis = max.sub(min, new Vector3d()).maxComponent();
            Arrays.sort(faces, from, to, Comparator.comparingDouble(t -> t.center.get(axis)));
            int mid = (from + to) >>> 1;
            left = new Branch(faces, from, mid, checkActive);
            right = new Branch(faces, mid, to, checkActive);
        }
    }

    public static Measurement measure(Surface jaw, Surface object, Runnable checkActive) {
        Search search = new Search(jaw, object, checkActive);
        search.visit(jaw.root, object.root);
        boolean overlap = search.crossing;
        if (!overlap && jaw.root.distanceSquared(object.root) == 0) {
            overlap = insideAny(jaw, object, search.eps, checkActive) || insideAny(object, jaw, search.eps, checkActive);
        }
        double distance = Math.sqrt(search.best);
        String state = overlap ? "penetrating" : distance <= search.eps ? "contact" : "separated";
        return new Measurement(state, distance, search.eps, search.jawFace, search.objectFace, search.jawPoint, search.objectPoint);
    }

    private static final class Search {
        final Surface jaw, object;
        final Runnable check;
        final double eps;
        double best = Double.POSITIVE_INFINITY;
        double alignment = -1;
        final Vector3d centerDirection;
        boolean crossing;
        Triangle jawFace, objectFace;
        final Vector3d jawPoint = new Vector3d(), objectPoint = new Vector3d();
        final Vector3d onEdge = new Vector3d(), onFace = new Vector3d();
        Search(Surface jaw, Surface object, Runnable check) {
            this.jaw = jaw;
            this.object = object;
            this.check = check;
            eps = Math.max(1e-12, Math.min(jaw.root.size(), object.root.size()) * 1e-8);
            centerDirection = object.root.min.add(object.root.max, new Vector3d()).sub(jaw.root.min).sub(jaw.root.max);
            if (centerDirection.lengthSquared() > 0) centerDirection.normalize();
        }
        void visit(Branch a, Branch b) {
            if ((crossing && best == 0) || a.distanceSquared(b) > best) return;
            check.run();
            if (a.left != null && (b.left == null || a.size() >= b.size())) {
                boolean leftFirst = a.left.distanceSquared(b) <= a.right.distanceSquared(b);
                visit(leftFirst ? a.left : a.right, b);
                visit(leftFirst ? a.right : a.left, b);
            } else if (b.left != null) {
                boolean leftFirst = b.left.distanceSquared(a) <= b.right.distanceSquared(a);
                visit(a, leftFirst ? b.left : b.right);
                visit(a, leftFirst ? b.right : b.left);
            } else {
                for (int i=a.from; i<a.to; i++) for (int j=b.from; j<b.to; j++) {
                    testEdges(jaw.faces[i], object.faces[j], false);
                    testEdges(object.faces[j], jaw.faces[i], true);
                    if (crossing && best == 0) return;
                }
            }
        }
        void testEdges(Triangle source, Triangle target, boolean reverse) {
            for (int i=0; i<3; i++) {
                Vector3d u = source.vertex(i), v = source.vertex((i+1)%3);
                double du = planeDistance(u, target), dv = planeDistance(v, target);
                if ((du > 0 && dv < 0) || (du < 0 && dv > 0)) {
                    u.lerp(v, du / (du-dv), onEdge);
                    Intersectiond.findClosestPointOnTriangle(target.a, target.b, target.c, onEdge, onFace);
                    if (onFace.distanceSquared(onEdge) <= eps*eps) {
                        consider(source, target, reverse, onEdge, onFace);
                        if (Math.min(Math.abs(du), Math.abs(dv)) > eps && interiorOfFace(onEdge, target, eps)
                                && (jaw.closed || object.closed)) {
                            crossing = true;
                        }
                    }
                }
                if (best == 0 && alignment >= 2 - 1e-12) continue;
                Intersectiond.findClosestPointOnTriangle(target.a, target.b, target.c, u, onFace);
                consider(source, target, reverse, u, onFace);
                for (int j=0; j<3; j++) {
                    closestEdges(u, v, target.vertex(j), target.vertex((j+1)%3), onEdge, onFace);
                    consider(source, target, reverse, onEdge, onFace);
                }
            }
        }
        void consider(Triangle source, Triangle target, boolean reversed, Vector3d p, Vector3d q) {
            double distance = p.distanceSquared(q);
            if (distance > best) return;
            // Edge/corner witnesses can belong to several faces. Prefer the opposing faces
            // along the separation direction when they give the same minimum distance.
            Vector3d direction = distance > eps*eps ? q.sub(p,new Vector3d()).normalize() : centerDirection;
            double score = Math.abs(direction.dot(source.normal)) + Math.abs(direction.dot(target.normal));
            if (distance == best && score <= alignment) return;
            best = distance;
            alignment = score;
            jawPoint.set(reversed ? q : p);
            objectPoint.set(reversed ? p : q);
            jawFace = reversed ? target : source;
            objectFace = reversed ? source : target;
        }
    }

    private static double planeDistance(Vector3d p, Triangle t) {
        return (p.x-t.a.x)*t.normal.x + (p.y-t.a.y)*t.normal.y + (p.z-t.a.z)*t.normal.z;
    }

    private static boolean interiorOfFace(Vector3d p, Triangle face, double eps) {
        for (int i=0; i<3; i++) {
            Vector3d a = face.vertex(i), edge = face.vertex((i+1)%3).sub(a, new Vector3d());
            if (edge.cross(p.sub(a, new Vector3d()), new Vector3d()).dot(face.normal) <= eps * edge.length()) return false;
        }
        return true;
    }

    // Segment/segment witnesses, with a relative parallelism test for millimetre-sized edges.
    private static void closestEdges(Vector3d p, Vector3d q, Vector3d a, Vector3d b, Vector3d x, Vector3d y) {
        double ux=q.x-p.x, uy=q.y-p.y, uz=q.z-p.z, vx=b.x-a.x, vy=b.y-a.y, vz=b.z-a.z;
        double wx=p.x-a.x, wy=p.y-a.y, wz=p.z-a.z;
        double uu=ux*ux+uy*uy+uz*uz, vv=vx*vx+vy*vy+vz*vz, uv=ux*vx+uy*vy+uz*vz;
        double uw=ux*wx+uy*wy+uz*wz, vw=vx*wx+vy*wy+vz*wz;
        double det=uu*vv-uv*uv;
        double s=det > 1e-15*uu*vv ? Math.clamp((uv*vw-vv*uw)/det, 0, 1) : 0;
        double t=(uv*s+vw)/vv;
        if (t < 0) { t=0; s=Math.clamp(-uw/uu,0,1); }
        if (t > 1) { t=1; s=Math.clamp((uv-uw)/uu,0,1); }
        p.lerp(q,s,x);
        a.lerp(b,t,y);
    }

    // Surface intersections alone miss nested solids and coincident faces. Check strict
    // interior with ray parity; boundary points themselves do not establish penetration.
    private static boolean insideAny(Surface source, Surface solid, double eps, Runnable check) {
        if (!solid.closed) return false;
        for (Triangle t : source.faces) {
            check.run();
            if (inside(t.center, solid, eps) || inside(t.a, solid, eps)
                    || inside(t.b, solid, eps) || inside(t.c, solid, eps)) return true;
            if (source.closed) {
                for (double sign : new double[]{-1,1}) {
                    Vector3d probe = new Vector3d(t.center).fma(sign * 4*eps, t.normal);
                    if (inside(probe, solid, eps) && inside(probe, source, eps)) return true;
                }
            }
        }
        return false;
    }

    private static final Vector3d RAY = new Vector3d(1,.371390676354,.694746590607).normalize();
    private static boolean inside(Vector3d p, Surface solid, double eps) {
        Branch box = solid.root;
        for (int i=0; i<3; i++) if (p.get(i) <= box.min.get(i)+eps || p.get(i) >= box.max.get(i)-eps) return false;
        var hits = new java.util.ArrayList<Double>();
        if (rayHits(p, solid, box, eps, hits)) return false;
        hits.sort(Double::compare);
        int crossings = 0;
        double previous = Double.NEGATIVE_INFINITY;
        for (double t : hits) {
            if (t-previous > eps) crossings++;
            previous = t;
        }
        return (crossings & 1) != 0;
    }

    private static boolean rayHits(Vector3d p, Surface solid, Branch box, double eps, List<Double> hits) {
        double near = -eps, far = Double.POSITIVE_INFINITY;
        for (int i=0; i<3; i++) {
            near = Math.max(near,(box.min.get(i)-eps-p.get(i))/RAY.get(i));
            far = Math.min(far,(box.max.get(i)+eps-p.get(i))/RAY.get(i));
        }
        if (near > far) return false;
        if (box.left != null) return rayHits(p,solid,box.left,eps,hits) || rayHits(p,solid,box.right,eps,hits);
        for (int i=box.from; i<box.to; i++) {
            Triangle t = solid.faces[i];
            if (Math.abs(planeDistance(p,t)) <= eps) {
                Vector3d nearest = new Vector3d();
                Intersectiond.findClosestPointOnTriangle(t.a,t.b,t.c,p,nearest);
                if (p.distanceSquared(nearest) <= eps*eps) return true;
            }
            double time = Intersectiond.intersectRayTriangle(p,RAY,t.a,t.b,t.c,1e-30);
            if (time > eps) hits.add(time);
        }
        return false;
    }
}
