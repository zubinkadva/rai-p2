package geometry;

//Represents a division of a 2D plane. This division is represented by two ordered points on the plane.
//A point is in the division if it is clockwise from the two points, in order.
//This divider accepts points that are on the dividing line.
public class PlaneDivider2 {
    private final Vector2 a;
    private final Vector2 b;
    
    public PlaneDivider2(Vector2 a, Vector2 b) {
        this.a = a;
        this.b = b;
    }
    
    public static PlaneDivider2 fromRayPair(Ray rayA, Ray rayB, Plane plane) {
        Vector3 aIntersect3 = rayA.intersectionPoint(plane);
        Vector3 bIntersect3 = rayB.intersectionPoint(plane);
        if (aIntersect3 == null || bIntersect3 == null) {
            if (aIntersect3 == null && bIntersect3 == null) {
                return new InfinitePlaneDivider();
            } else {
                if (aIntersect3 == null) aIntersect3 = lerpToward(rayA, rayB, plane).intersectionPoint(plane);
                if (bIntersect3 == null) bIntersect3 = lerpToward(rayB, rayA, plane).intersectionPoint(plane);
                return new PlaneDivider2(aIntersect3.asVector2(), bIntersect3.asVector2());
            }
        } else {
            return new PlaneDivider2(aIntersect3.asVector2(), bIntersect3.asVector2());
        }
    }
    
    public static Ray lerpToward(Ray toMove, Ray reference, Plane target) {
        while (toMove.intersectionPoint(target) == null) {
            toMove = new Ray(toMove.origin, toMove.direction.add(reference.direction).div(2));
        }
        return toMove;
    }
    
    public boolean contains(Vector2 point) {
        return isClockwise(a, b, point);
    }
    
    //Checks clockwise-ness using the cross product. (Only some of the calculations are needed.)
    //Inspired by http://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
    public static boolean isClockwise(Vector2 pointA, Vector2 pointB, Vector2 pointC) {
        Vector2 vecA = pointC.sub(pointB);
        Vector2 vecB = pointA.sub(pointB);
        double crossNorm = vecA.x * vecB.y - vecA.y * vecB.x;
        return crossNorm <= 0;
    }
}
