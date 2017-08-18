package geometry;

public class Plane {
    public final Vector3 origin;
    public final Vector3 norm;
    
    public Plane(Vector3 origin, Vector3 norm) {
        this.origin = origin;
        this.norm = norm;
    }
}
