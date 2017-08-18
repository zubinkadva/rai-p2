package planner;

import java.util.Arrays;

import commands.ConfigurationCoordinate;

public class ScoredActingRange implements Comparable<ScoredActingRange> {
    public double[][] actingRange;
    public double expectedUtility;
    
    public ScoredActingRange(double[][] actingRange, double expectedUtility) {
        this.actingRange = actingRange;
        this.expectedUtility = expectedUtility;
    }
    
    public PlannedAction getPlannedAction() {
        return new PlannedAction(getCenterOfActingRange(actingRange), expectedUtility);
    }
    
    public ConfigurationCoordinate getCenter() {
        return getCenterOfActingRange(actingRange);
    }
    
    public static ConfigurationCoordinate getCenterOfActingRange(double[][] actingRange) {
        return new ConfigurationCoordinate(average(actingRange[0]), average(actingRange[1]), average(actingRange[2]), average(actingRange[3]), average(actingRange[4]));
    }
    
    public static double average(double[] arr) {
        return Arrays.stream(arr).average().getAsDouble();
    }
    
    @Override
    public int compareTo(ScoredActingRange other) {
        return Double.compare(expectedUtility, other.expectedUtility);
    }
}
