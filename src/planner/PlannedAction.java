package planner;

import commands.ConfigurationCoordinate;

public class PlannedAction implements Comparable<PlannedAction> {
    public ConfigurationCoordinate plannedCommand;
    public double expectedUtility;
    
    public PlannedAction(ConfigurationCoordinate plannedCommand, double expectedUtility) {
        this.plannedCommand = plannedCommand;
        this.expectedUtility = expectedUtility;
    }
    
    @Override
    public int compareTo(PlannedAction other) {
        return Double.compare(expectedUtility, other.expectedUtility);
    }
}
