package cv;

import geometry.InfinitePlaneDivider;
import geometry.PlaneDivider2;
import geometry.Vector2;

public class VisibleRange {
    public final PlaneDivider2[] planeDividers;
    private boolean allInfinite;
    
    public VisibleRange(PlaneDivider2[] planeDividers) {
        this.planeDividers = planeDividers;
        allInfinite = true;
        for (PlaneDivider2 planeDivider : planeDividers) if (!(planeDivider instanceof InfinitePlaneDivider)) allInfinite = false;
    }
    
    //A point is in a (convex) region if it is on the correct side of every line bounding that region.
    public boolean contains(Vector2 point) {
        if (allInfinite) return false;
        for (PlaneDivider2 divider: planeDividers) if (!divider.contains(point)) return false;
        return true;
    }
    
    public boolean contains(double[] position) {
        return contains(new Vector2(position[1], position[0]));
    }
    
    public boolean contains(double y, double x) {
        return contains(new Vector2(x, y));
    }
}
