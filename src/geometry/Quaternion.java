package geometry;

//This Quaternion implementation was adapted from the example provided at http://introcs.cs.princeton.edu/java/32class/Quaternion.java.html
//Further functionality was added as needed.

//This class was only implemented once certain geometric problems became difficult to solve with ad-hoc solutions. (Notably, the method to get the visible range of a camera.)
//Because of this, certain earlier methods that could have taken advantage of this library and become more clear do not. (Notably, the method for converting from configuration space to the workspace.)
public class Quaternion {
    public static final Quaternion IDENTITY = new Quaternion(1, 0, 0, 0);
    
    public final double x0, x1, x2, x3; 

    // create a new object with the given components
    public Quaternion(double x0, double x1, double x2, double x3) {
        this.x0 = x0;
        this.x1 = x1;
        this.x2 = x2;
        this.x3 = x3;
    }

    // return a string representation of the invoking object
    public String toString() {
        return x0 + " + " + x1 + "i + " + x2 + "j + " + x3 + "k";
    }

    // return the quaternion norm
    public double norm() {
        return Math.sqrt(x0*x0 + x1*x1 +x2*x2 + x3*x3);
    }

    // return the quaternion conjugate
    public Quaternion conjugate() {
        return new Quaternion(x0, -x1, -x2, -x3);
    }

    // return a new Quaternion whose value is (this + b)
    public Quaternion add(Quaternion b) {
        Quaternion a = this;
        return new Quaternion(a.x0+b.x0, a.x1+b.x1, a.x2+b.x2, a.x3+b.x3);
    }

    // return a new Quaternion whose value is (this * b)
    public Quaternion rawMult(Quaternion b) {
        Quaternion a = this;
        double y0 = a.x0*b.x0 - a.x1*b.x1 - a.x2*b.x2 - a.x3*b.x3;
        double y1 = a.x0*b.x1 + a.x1*b.x0 + a.x2*b.x3 - a.x3*b.x2;
        double y2 = a.x0*b.x2 - a.x1*b.x3 + a.x2*b.x0 + a.x3*b.x1;
        double y3 = a.x0*b.x3 + a.x1*b.x2 - a.x2*b.x1 + a.x3*b.x0;
        return new Quaternion(y0, y1, y2, y3);
    }
    
    public Quaternion mult(Quaternion b) {
        return this.rawMult(b).normalized();
    }

    /*// return a new Quaternion whose value is the inverse of this
    //Since I'm always working with normalized quaternions, this shouldn't be any different (effectively) from conjugate();
    public Quaternion inverse() {
        double d = x0*x0 + x1*x1 + x2*x2 + x3*x3;
        return new Quaternion(x0/d, -x1/d, -x2/d, -x3/d);
    }*/
    
    public Quaternion normalized() {
        double norm = norm();
        return new Quaternion(x0 / norm, x1 / norm, x2 / norm, x3 / norm);
    }
    
    public static Quaternion fromAxisAngle(Vector3 axis, double angle) {
        axis = axis.normalized();
        
        double sin = Math.sin(angle / 2);
        double cos = Math.cos(angle / 2);
        return new Quaternion(cos, axis.x * sin, axis.y * sin, axis.z * sin).normalized();
    }
    
    //Inspiration taken from http://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
    //and http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
    public static Quaternion fromFromTo(Vector3 from, Vector3 to) {
        from = from.normalized();
        to = to.normalized();
        Vector3 cross = from.cross(to);
        if (cross.squareNorm() < 0.000000001) {
            return Quaternion.IDENTITY;
        }
        else {
            double dot = from.dot(to);
            double angle = Math.acos(dot);
            return Quaternion.fromAxisAngle(cross, angle);
        }
    }
    
    //Assumes that x0 ~= 0. This should generally only be used to get back a Vector3 after using Quaternion multiplication to rotate it.
    public Vector3 asVector3() {
        return new Vector3(x1, x2, x3);
    }
}