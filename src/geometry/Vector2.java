package geometry;

public class Vector2 {
    public final double x;
    public final double y;
    
    public Vector2 (double x, double y) {
        this.x = x;
        this.y = y;
    }
    
    public Vector2 add(Vector2 other) {
        return new Vector2(x + other.x, y + other.y);
    }
    
    public Vector2 sub(Vector2 other) {
        return this.add(other.mult(-1));
    }
    
    public Vector2 mult(double scalar) {
        return new Vector2(x * scalar, y * scalar);
    }
    
    public Vector2 div(double scalar) {
        return this.mult(1 / scalar);
    }
    
    public double norm() {
        return Math.sqrt(x * x + y * y);
    }
    
    public double distanceTo(Vector2 other) {
        return this.sub(other).norm();
    }
    
    @Override
    public String toString() {
        return String.format("[%f, %f]", x, y);
    }
}
