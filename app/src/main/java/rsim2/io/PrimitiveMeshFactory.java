package rsim2.io;

import rsim2.graphics.Mesh;

public final class PrimitiveMeshFactory {
    private static final int CYLINDER_SEGMENTS = 32;

    private PrimitiveMeshFactory() {}

    public static Mesh box(float halfX, float halfY, float halfZ) {
        requirePositive(halfX, halfY, halfZ);
        float[][] corners = {
                {-halfX, -halfY, -halfZ}, {halfX, -halfY, -halfZ},
                {halfX, halfY, -halfZ}, {-halfX, halfY, -halfZ},
                {-halfX, -halfY, halfZ}, {halfX, -halfY, halfZ},
                {halfX, halfY, halfZ}, {-halfX, halfY, halfZ}
        };
        int[][] faces = {{0,3,2,1}, {4,5,6,7}, {0,1,5,4},
                {3,7,6,2}, {0,4,7,3}, {1,2,6,5}};
        Builder builder = new Builder(12);
        for (int[] face : faces) {
            builder.triangle(corners[face[0]], corners[face[1]], corners[face[2]]);
            builder.triangle(corners[face[0]], corners[face[2]], corners[face[3]]);
        }
        Mesh mesh = builder.build();
        mesh.setSourcePath("primitive:box:" + halfX + ":" + halfY + ":" + halfZ);
        return mesh;
    }

    public static Mesh cylinder(float radius, float halfLength) {
        requirePositive(radius, halfLength);
        Builder builder = new Builder(CYLINDER_SEGMENTS * 4);
        for (int i = 0; i < CYLINDER_SEGMENTS; i++) {
            double a = i * 2.0 * Math.PI / CYLINDER_SEGMENTS;
            double b = (i + 1) * 2.0 * Math.PI / CYLINDER_SEGMENTS;
            float x0 = radius * (float) Math.cos(a), y0 = radius * (float) Math.sin(a);
            float x1 = radius * (float) Math.cos(b), y1 = radius * (float) Math.sin(b);
            float[] bottom0 = {x0, y0, -halfLength}, bottom1 = {x1, y1, -halfLength};
            float[] top0 = {x0, y0, halfLength}, top1 = {x1, y1, halfLength};
            builder.triangle(bottom0, bottom1, top1);
            builder.triangle(bottom0, top1, top0);
            builder.triangle(new float[]{0, 0, halfLength}, top0, top1);
            builder.triangle(new float[]{0, 0, -halfLength}, bottom1, bottom0);
        }
        Mesh mesh = builder.build();
        mesh.setSourcePath("primitive:cylinder:" + radius + ":" + halfLength);
        return mesh;
    }

    private static void requirePositive(float... dimensions) {
        for (float dimension : dimensions) {
            if (!Float.isFinite(dimension) || dimension <= 0) {
                throw new IllegalArgumentException("Primitive dimensions must be finite and positive");
            }
        }
    }

    private static final class Builder {
        private final float[] vertices;
        private final float[] normals;
        private final int[] indices;
        private int offset;

        Builder(int triangles) {
            vertices = new float[triangles * 9];
            normals = new float[triangles * 9];
            indices = new int[triangles * 3];
        }

        void triangle(float[] a, float[] b, float[] c) {
            float ux=b[0]-a[0], uy=b[1]-a[1], uz=b[2]-a[2];
            float vx=c[0]-a[0], vy=c[1]-a[1], vz=c[2]-a[2];
            float nx=uy*vz-uz*vy, ny=uz*vx-ux*vz, nz=ux*vy-uy*vx;
            float length=(float)Math.sqrt(nx*nx+ny*ny+nz*nz);
            for (float[] vertex : new float[][]{a,b,c}) {
                indices[offset/3]=offset/3;
                vertices[offset]=vertex[0]; normals[offset++]=nx/length;
                vertices[offset]=vertex[1]; normals[offset++]=ny/length;
                vertices[offset]=vertex[2]; normals[offset++]=nz/length;
            }
        }

        Mesh build() { return new Mesh(vertices, normals, indices); }
    }
}
