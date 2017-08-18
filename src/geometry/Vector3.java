package geometry;

public class Vector3 {
    public static final Vector3 ZERO = new Vector3(0, 0, 0);
    public static final Vector3 UP = new Vector3(0, 0, 1);
    public static final Vector3 DOWN = new Vector3(0, 0, -1);
    public static final Vector3 LEFT = new Vector3(-1, 0, 0);
    public static final Vector3 RIGHT = new Vector3(1, 0, 0);
    public static final Vector3 FORWARD = new Vector3(0, 1, 0);
    public static final Vector3 BACK = new Vector3(0, -1, 0);
    
    public final double x;
    public final double y;
    public final double z;
    
    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    
    public double norm() {
        return Math.sqrt(x * x + y * y + z * z);
    }
    
    public double squareNorm() {
        return x * x + y * y + z * z;
    }
    
    public Vector3 normalized() {
        double norm = norm();
        return new Vector3(x / norm, y / norm, z / norm);
    }
    
    public Quaternion asQuat() {
        return new Quaternion(0, x, y, z);
    }
    
    public Vector3 afterRotation(Quaternion quat) {
        return quat.rawMult(this.asQuat()).rawMult(quat.conjugate()).asVector3();
    }
    
    public Vector3 add(Vector3 other) {
        return new Vector3(this.x + other.x, this.y + other.y, this.z + other.z);
    }
    
    public Vector3 sub(Vector3 other) {
        return this.add(other.mult(-1));
    }
    
    public double dot(Vector3 other) {
        return x * other.x + y * other.y + z * other.z;
    }
    
    public Vector3 cross(Vector3 other) {
        return new Vector3(
                y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x
                );
    }
    
    public Vector3 mult(double scalar) {
        return new Vector3(x * scalar, y * scalar, z * scalar);
    }
    
    public Vector3 div(double scalar) {
        return this.mult(1 / scalar);
    }
    
    public double distanceTo(Vector3 other) {
        return this.sub(other).norm();
    }
    
    public Vector3 withScale(double scale) {
        double norm = this.norm();
        double ratio = scale / norm;
        return new Vector3(x * ratio, y * ratio, z * ratio);
    }
    
    public Vector2 asVector2() {
        return new Vector2(x, y);
    }
    
    @Override
    public String toString() {
        return String.format("[%f, %f, %f]", x, y, z);
    }
}
