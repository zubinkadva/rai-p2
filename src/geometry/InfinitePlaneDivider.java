package geometry;

//When both rays in a ray pair don't intersect the table, the result is a bounding line at infinity, which accepts all points.
public class InfinitePlaneDivider extends PlaneDivider2 {
    public InfinitePlaneDivider() {super(null, null);}
    
    @Override
    public boolean contains(Vector2 point) {
        return true;
    }
}
