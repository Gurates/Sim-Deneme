package rsim2.mcp;

import com.google.gson.JsonArray;
import org.joml.Matrix4d;
import org.joml.Vector3d;
import org.joml.Vector3f;
import rsim2.collision.*;
import rsim2.graphics.Mesh;
import rsim2.scene.SceneNode;
import java.util.*;

final class McpSurfaceGeometry {
    private McpSurfaceGeometry() {}

    record Part(String node, Integer mesh, String geometry, List<Integer> path, SurfaceProximity.Surface surface) {}

    static JsonArray xyz(Vector3d p) {
        JsonArray v = new JsonArray();
        v.add(p.x); v.add(p.y); v.add(p.z);
        return v;
    }

    static void collect(SceneNode node, Integer meshIndex, Set<Integer> triangles, List<Integer> collisionPath,
                        Matrix4d world, List<Part> parts) {
        McpBridge.ensureRequestActive();
        if (collisionPath != null || (meshIndex == null && node.getMeshes().isEmpty())) {
            CollisionShape shape = node.getCollisionShape();
            List<Integer> path = collisionPath == null ? List.of() : collisionPath;
            for (int i : path) {
                if (!(shape instanceof CompoundShape compound) || i >= compound.getChildren().size())
                    throw new IllegalArgumentException("Invalid collision_path on " + node.getId());
                var child = compound.getChildren().get(i);
                world.mul(new Matrix4d(child.getLocalTransform()));
                shape = child.getShape();
            }
            if (shape != null) collision(node.getId(),shape,world,path,parts);
            else if (collisionPath != null) throw new IllegalArgumentException("Missing collision geometry on " + node.getId());
            return;
        }
        int start = meshIndex == null ? 0 : meshIndex;
        int end = meshIndex == null ? node.getMeshes().size() : start+1;
        if (start >= node.getMeshes().size()) throw new IllegalArgumentException("Unknown mesh_index on " + node.getId());
        for (int m=start; m<end; m++) {
            Mesh mesh = node.getMeshes().get(m);
            int[] indices = mesh.getIndices();
            float[] vertices = mesh.getVertices();
            if (indices == null || indices.length == 0 || indices.length%3 != 0)
                throw new IllegalArgumentException("Mesh has no indexed triangle surface: " + node.getId());
            int count = indices.length/3;
            for (int f : triangles) if (f >= count) throw new IllegalArgumentException("Unknown triangle_index: " + f);
            List<SurfaceProximity.Triangle> faces = new ArrayList<>();
            for (int f=0; f<count; f++) {
                if ((f & 255) == 0) McpBridge.ensureRequestActive();
                if (!triangles.isEmpty() && !triangles.contains(f)) continue;
                Vector3d a = vertex(vertices,indices[3*f],world);
                Vector3d b = vertex(vertices,indices[3*f+1],world);
                Vector3d c = vertex(vertices,indices[3*f+2],world);
                faces.add(new SurfaceProximity.Triangle(f,a,b,c));
            }
            parts.add(new Part(node.getId(),m,"mesh",null,new SurfaceProximity.Surface(faces,McpBridge::ensureRequestActive)));
        }
    }

    private static Vector3d vertex(float[] vertices, int i, Matrix4d world) {
        if (i < 0 || i >= vertices.length/3) throw new IllegalArgumentException("Invalid mesh vertex index.");
        return world.transformPosition(new Vector3d(vertices[3*i],vertices[3*i+1],vertices[3*i+2]));
    }


    private static final int[][] QUADS = {{0,4,6,2},{1,3,7,5},{0,1,5,4},{2,6,7,3},{0,2,3,1},{4,5,7,6}};

    private static void collision(String node, CollisionShape shape, Matrix4d world, List<Integer> path, List<Part> parts) {
        if (shape instanceof CompoundShape compound) {
            int index = 0;
            for (var child : compound.getChildren()) {
                List<Integer> nested = new ArrayList<>(path);
                nested.add(index++);
                collision(node,child.getShape(),new Matrix4d(world).mul(new Matrix4d(child.getLocalTransform())),nested,parts);
            }
            return;
        }
        List<SurfaceProximity.Triangle> faces = new ArrayList<>();
        String kind;
        if (shape instanceof BoxShape) {
            Vector3f lo = shape.getLocalAABB().getMin(), hi = shape.getLocalAABB().getMax();
            Vector3d[] p = new Vector3d[8];
            for (int i=0; i<8; i++) p[i]=world.transformPosition(new Vector3d(
                    (i&1)==0 ? lo.x : hi.x,(i&2)==0 ? lo.y : hi.y,(i&4)==0 ? lo.z : hi.z));
            for (int[] q : QUADS) quad(faces,p[q[0]],p[q[1]],p[q[2]],p[q[3]]);
            kind = "collision_box";
        } else if (shape instanceof ConvexMeshShape convex) {
            List<Vector3d> points = convex.getVertices().stream()
                    .map(p -> world.transformPosition(new Vector3d(p))).toList();
            faces = SurfaceProximity.convexHull(points, McpBridge::ensureRequestActive);
            kind = "collision_convex_hull";
        } else {
            kind = curved(shape,world,faces);
        }
        parts.add(new Part(node,null,kind,path,new SurfaceProximity.Surface(faces,McpBridge::ensureRequestActive)));
    }

    private static void quad(List<SurfaceProximity.Triangle> faces, Vector3d a, Vector3d b, Vector3d c, Vector3d d) {
        faces.add(new SurfaceProximity.Triangle(faces.size(),a,b,c));
        faces.add(new SurfaceProximity.Triangle(faces.size(),a,c,d));
    }

    /** Inscribed polygon surface for collision-only curved primitives. Labelled in every witness. */
    private static String curved(CollisionShape shape, Matrix4d world, List<SurfaceProximity.Triangle> faces) {
        Vector3d p0=new Vector3d(), p1=new Vector3d();
        double radius;
        String kind;
        if (shape instanceof SphereShape sphere) {
            p0.set(sphere.getLocalCenter()); p1.set(p0); radius=sphere.getRadius();
            kind="tessellated_collision_sphere";
        } else if (shape instanceof CapsuleShape capsule) {
            p0.set(capsule.getLocalP0()); p1.set(capsule.getLocalP1()); radius=capsule.getRadius();
            kind="tessellated_collision_capsule";
        } else if (shape instanceof CylinderShape cylinder) {
            p0.z=-cylinder.getHeight()/2; p1.z=-p0.z; radius=cylinder.getRadius();
            kind="tessellated_collision_cylinder";
        } else {
            throw new IllegalArgumentException("Collision geometry has no indexed surface: " + shape.getClass().getSimpleName());
        }
        Vector3d axis=p1.sub(p0,new Vector3d());
        if (axis.lengthSquared()==0) axis.set(0,0,1); else axis.normalize();
        Vector3d x=axis.cross(Math.abs(axis.x)<.9 ? new Vector3d(1,0,0) : new Vector3d(0,1,0),new Vector3d()).normalize();
        Vector3d y=axis.cross(x,new Vector3d());
        List<Vector3d[]> rings=new ArrayList<>();
        boolean flat=shape instanceof CylinderShape;
        for (int cap=0; cap<2; cap++) {
            Vector3d center=cap==0 ? p0 : p1;
            int steps=flat ? 1 : 8;
            for (int k=0; k<=steps; k++) {
                if (cap==1 && k==0 && p0.equals(p1)) continue;
                double angle=(cap-1 + (double)k/steps)*Math.PI/2;
                boolean pole=cap==0 && k==0 || cap==1 && k==steps;
                double r=pole ? 0 : radius*Math.cos(angle);
                Vector3d base=new Vector3d(center).fma(flat ? 0 : radius*Math.sin(angle),axis);
                Vector3d[] ring=new Vector3d[32];
                for (int i=0; i<ring.length; i++) {
                    double phi=i*2*Math.PI/ring.length;
                    ring[i]=world.transformPosition(new Vector3d(base).fma(r*Math.cos(phi),x).fma(r*Math.sin(phi),y));
                }
                rings.add(ring);
            }
        }
        for (int k=1; k<rings.size(); k++) {
            Vector3d[] a=rings.get(k-1), b=rings.get(k);
            for (int i=0; i<a.length; i++) {
                int j=(i+1)%a.length;
                quad(faces,a[i],a[j],b[j],b[i]);
            }
        }
        return kind;
    }
}
