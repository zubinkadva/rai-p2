package geometry;

public class Ray {
    public final Vector3 origin;
    public final Vector3 direction;
    
    public Ray(Vector3 origin, Vector3 direction) {
        this.origin = origin;
        this.direction = direction;
    }
    
    //Mathematics from https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-plane-and-ray-disk-intersection
    public Vector3 intersectionPoint(Plane plane) {
        double denom = this.direction.dot(plane.norm);
        if (Math.abs(denom) < 0.000000001) {
            return null;
        } else {
            double numer = plane.origin.sub(this.origin).dot(plane.norm);
            double intersectionDistance = numer / denom;
            if (intersectionDistance >= 0) {
                return origin.add(direction.withScale(intersectionDistance));
            } else {
                return null;
            }
        }
    }
}
