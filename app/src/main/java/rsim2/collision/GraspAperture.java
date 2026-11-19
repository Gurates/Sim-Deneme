package rsim2.collision;

import org.joml.Intersectiond;
import org.joml.Vector3d;
import rsim2.collision.SurfaceProximity.Surface;
import rsim2.collision.SurfaceProximity.Triangle;

import java.util.List;
import java.util.Optional;

public final class GraspAperture {
    private GraspAperture() {}

    public record Section(Vector3d point, Vector3d axis) {
        public Section {
            if (!point.isFinite() || !axis.isFinite() || !(axis.lengthSquared() > 0))
                throw new IllegalArgumentException("A grasp section needs a finite point and a nonzero direction.");
            axis = new Vector3d(axis).normalize();
        }
    }

    public record Measurement(Vector3d fixedPoint, Vector3d movingPoint, Vector3d axis, double signedOpening,
                              double objectMin, double objectMax, double tolerance) {
        public Vector3d center() { return fixedPoint.lerp(movingPoint, 0.5, new Vector3d()); }
        public double objectWidth() { return objectMax - objectMin; }
        public double objectCenterOffset() { return (objectMin + objectMax) * 0.5; }
        public boolean widthFits() { return signedOpening >= 0 && objectWidth() <= signedOpening + tolerance; }
        public boolean withinOpening() {
            return widthFits() && objectMin >= -signedOpening * 0.5 - tolerance
                    && objectMax <= signedOpening * 0.5 + tolerance;
        }
    }

    private record Hit(double position, Vector3d normal, double tolerance) {}
    private record Span(Hit low, Hit high) {}
    private record Pad(Vector3d center, Vector3d parallelNormal, double size) {}

    public static Optional<Measurement> measure(List<Surface> fixed, List<Surface> moving, List<Surface> object,
                                                Section section, Runnable checkActive) {
        if (section == null) {
            section = inferSection(fixed, moving, checkActive);
            if (section == null) return Optional.empty();
        }

        Span fixedHits = lineHits(fixed, section, checkActive);
        Span movingHits = lineHits(moving, section, checkActive);
        if (fixedHits.low == null || movingHits.low == null) return Optional.empty();
        // Orient the reported axis from fixed to moving, whichever sign the caller used.
        boolean reversed = fixedHits.low.position + fixedHits.high.position > movingHits.low.position + movingHits.high.position;
        Hit left = reversed ? fixedHits.low : fixedHits.high;
        Hit right = reversed ? movingHits.high : movingHits.low;
        Vector3d axis = reversed ? section.axis.negate(new Vector3d()) : section.axis;
        Vector3d a = new Vector3d(section.point).fma(left.position, section.axis);
        Vector3d b = new Vector3d(section.point).fma(right.position, section.axis);
        double tolerance = coordinateTolerance(a, b) + Math.min(left.tolerance, right.tolerance);
        double gap = reversed ? left.position - right.position : right.position - left.position;
        if (Math.abs(gap) <= tolerance) gap = 0;

        // The target's whole projected extent at its current orientation, including children.
        Vector3d center = a.lerp(b, 0.5, new Vector3d());
        double min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;
        Vector3d relative = new Vector3d();
        for (Surface part : object) {
            int n = 0;
            for (Triangle face : part.triangles()) {
                if ((n++ & 255) == 0) checkActive.run();
                for (int i = 0; i < 3; i++) {
                    double projection = face.vertex(i).sub(center, relative).dot(axis);
                    min = Math.min(min, projection);
                    max = Math.max(max, projection);
                }
            }
        }
        return Optional.of(new Measurement(a, b, axis, gap, min, max, tolerance));
    }

    private static Section inferSection(List<Surface> fixed, List<Surface> moving, Runnable checkActive) {
        Pad a = summarizePad(fixed, checkActive), b = summarizePad(moving, checkActive);
        Vector3d midpoint = a.center.lerp(b.center, 0.5, new Vector3d());
        Vector3d direction = b.center.sub(a.center, new Vector3d());
        // Explicit flat pads have a known normal even when they touch and their centers coincide.
        if (a.parallelNormal != null && b.parallelNormal != null
                && Math.abs(a.parallelNormal.dot(b.parallelNormal)) >= 1 - 1e-12)
            return new Section(midpoint, a.parallelNormal);
        double tolerance = coordinateTolerance(a.center, b.center) + 1e-12 * Math.min(a.size, b.size);
        if (direction.length() <= tolerance) return null;
        Section seed = new Section(midpoint, direction);
        Hit left = lineHits(fixed, seed, checkActive).high;
        Hit right = lineHits(moving, seed, checkActive).low;
        if (left == null || right == null) return null;
        // At this depth take the bisector of the two intersected faces. Each normal was
        // oriented along the seed, so opposite mesh winding cannot cancel the directions.
        return new Section(midpoint, new Vector3d(left.normal).add(right.normal));
    }

    private static double coordinateTolerance(Vector3d a, Vector3d b) {
        double scale = Math.max(Math.abs(a.get(a.maxComponent())), Math.abs(b.get(b.maxComponent())));
        return 32 * Math.ulp(scale);
    }

    private static Pad summarizePad(List<Surface> parts, Runnable checkActive) {
        Vector3d center = new Vector3d(), ab = new Vector3d(), ac = new Vector3d();
        Vector3d normal = parts.getFirst().triangles().getFirst().normal();
        double weight = 0;
        for (Surface part : parts) {
            int n = 0;
            for (Triangle t : part.triangles()) {
                if ((n++ & 255) == 0) checkActive.run();
                double area = t.b().sub(t.a(), ab).cross(t.c().sub(t.a(), ac)).length();
                if (normal != null && Math.abs(normal.dot(t.normal())) < 1 - 1e-12) normal = null;
                weight += area;
                center.lerp(t.center(), area / weight);
            }
        }
        return new Pad(center, normal, Math.sqrt(weight));
    }

    private static Span lineHits(List<Surface> parts, Section section, Runnable checkActive) {
        Hit low = null, high = null;
        Vector3d relative = new Vector3d(), point = new Vector3d(), closest = new Vector3d();
        for (Surface part : parts) {
            int n = 0;
            for (Triangle face : part.triangles()) {
                if ((n++ & 255) == 0) checkActive.run();
                double cosine = face.normal().dot(section.axis);
                if (Math.abs(cosine) < 1e-8) continue;
                double along = face.a().sub(section.point, relative).dot(face.normal()) / cosine;
                if (low != null && along > low.position && along < high.position) continue;
                section.point.fma(along, section.axis, point);
                Intersectiond.findClosestPointOnTriangle(face.a(), face.b(), face.c(), point, closest);
                double epsilon = coordinateTolerance(point, face.a()) + 1e-9 * Math.max(
                        face.a().distance(face.b()), face.a().distance(face.c()));
                if (point.distanceSquared(closest) > epsilon * epsilon) continue;
                Hit hit = new Hit(along, new Vector3d(face.normal()).mul(Math.signum(cosine)), epsilon);
                if (low == null || along < low.position || (along == low.position && Math.abs(cosine) > low.normal.dot(section.axis))) low = hit;
                if (high == null || along > high.position || (along == high.position && Math.abs(cosine) > high.normal.dot(section.axis))) high = hit;
            }
        }
        return new Span(low, high);
    }
}
