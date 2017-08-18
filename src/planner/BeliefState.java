package planner;

import java.util.Arrays;

import commands.ConfigurationCoordinate;
import commands.WorkspaceCoordinate;

import cv.CameraModel;
import cv.VisibleRange;

public class BeliefState implements Comparable<BeliefState> {
    public double[][] locationProbabilities;
    public double binLength; //The length, in robot units, of one side of a bin in locationProbabilities

    public double yCenter;
    public double xCenter;

    public double yOffset;
    public double xOffset;
    
    private double cachedStdev;
    
    private BeliefState(double[][] locationProbabilities, double binLength, double yCenter, double xCenter, double cachedStdev) {
        this.locationProbabilities = locationProbabilities;
        this.binLength = binLength;
        this.yCenter = yCenter;
        this.xCenter = xCenter;
        this.yOffset = yCenter - binLength * ((locationProbabilities.length / 2d) - 0.5d);
        this.xOffset = xCenter - binLength * ((locationProbabilities[0].length / 2d) - 0.5d);
        this.cachedStdev = cachedStdev;
    }
    
    public BeliefState(double[][] locationProbabilities, double binLength, double yCenter, double xCenter) {
        this(locationProbabilities, binLength, yCenter, xCenter, Double.NaN);
    }
    
    public BeliefState(BeliefState other) {
        this(copy(other.locationProbabilities), other.binLength, other.yCenter, other.xCenter, other.cachedStdev);
    }
    
    public static double[][] copy(double[][] original) {
        double[][] copy = new double[original.length][];
        for (int y = 0; y < original.length; y++) {
            copy[y] = original[y].clone();
        }
        return copy;
    }
    
    @Override
    public int compareTo(BeliefState other) {
        return Double.compare(getScore(), other.getScore());
    }
    
    @Override
    public boolean equals(Object o) {
        if (this.getClass().isInstance(o)) {
            BeliefState other = (BeliefState)o;
            if (binLength != other.binLength) return false;
            return Arrays.deepEquals(locationProbabilities, other.locationProbabilities);
        } else if (o.getClass().isInstance(this)) {
            return o.equals(this);
        } else return false;
    }
    
    public double getScore() {
        return -weightedStdev();
    }
    
    public double weightedStdev() {
        if (Double.isNaN(cachedStdev)) stdevUpdate();
        return cachedStdev;
    }
    
    public void stdevUpdate() {
        double[] center = weightedCenter();
        double variance = 0;
        for (int y = 0; y < locationProbabilities.length; y++) {
            double yDiff = y - center[0];
            double y2 = yDiff * yDiff;
            for (int x = 0; x < locationProbabilities[y].length; x++) {
                double xDiff = x - center[1];
                double x2 = xDiff * xDiff;
                variance += (y2 + x2) * locationProbabilities[y][x];
            }
        }
        cachedStdev = Math.sqrt(variance);
    }
    
    public double[] weightedCenter() {
        double[] center = new double[2];
        for (int y = 0; y < locationProbabilities.length; y++) {
            for (int x = 0; x < locationProbabilities[y].length; x++) {
                center[0] += y * locationProbabilities[y][x];
                center[1] += x * locationProbabilities[y][x];
            }
        }
        return center;
    }
    
    public double[] samplePosition() {
        double remainingProb = Math.random();
        for (int y = 0; y < locationProbabilities.length; y++) {
            for (int x = 0; x < locationProbabilities[y].length; x++) {
                remainingProb -= locationProbabilities[y][x];
                if (remainingProb < 0) return new double[] {yOffset + binLength * y, xOffset + binLength * x};
            }
        }
        
        //If we reached this point, its because the caller messed with the probability distribution themselves and didn't normalize it afterward. Normalize it and resample.
        normalize();
        return samplePosition();
    }
    
    //After profiling, I have determined that ~80% of the CPU time is spent in updateBelief and updateUnseen. Therefore, these are good candidates for parallelization.
    //For this reason, I'm coming back and optimizing these methods.
    
    /* Original code:
    public void updateBelief(double[] sampledPosition, ConfigurationCoordinate configuration, CameraModel cameraModel, double targetHeight) {
        double stdev = cameraModel.getUncertainty(configuration, sampledPosition, targetHeight);
        double twoSigSquared = 2 * stdev * stdev;
        for (int y = 0; y < locationProbabilities.length; y++) {
            for (int x = 0; x < locationProbabilities[y].length; x++) {
                double yPos = yOffset + binLength * y;
                double xPos = xOffset + binLength * x;
                double yDiff = yPos - sampledPosition[0];
                double xDiff = xPos - sampledPosition[1];
                double squareDist = xDiff * xDiff + yDiff * yDiff;
                locationProbabilities[y][x] = Math.exp(-squareDist / twoSigSquared);
            }
        }
        normalize();
    }
    
    //It's not exactly statistically rigorous, but it's reasonable.
    public void updateUnseen(VisibleRange visibleRange) {
        for (int y = 0; y < locationProbabilities.length; y++) {
            for (int x = 0; x < locationProbabilities[y].length; x++) {
                if (visibleRange.contains(yOffset + binLength * y, xOffset + binLength * x)) locationProbabilities[y][x] /= 10;
            }
        }
        normalize();
    }*/
    
    //The robot has a tendency to try to crane its arm at an interesting angle, putting the camera near the table and nearly perpendicular. This /would/ cover a lot of the table, except most of the view ends up being blocked by the robot arm itself. So, I added an explicit check for this condition to discourage that behavior. (And basically teach the algorithm that the arm isn't invisible.)
    
    public void updateBelief(double[] sampledPosition, ConfigurationCoordinate configuration, CameraModel cameraModel, double targetHeight) {
        WorkspaceCoordinate workspace = new WorkspaceCoordinate(configuration);
        boolean xLow = workspace.x <= 0;
        boolean yLow = workspace.y <= 0;
        boolean zLow = workspace.z <= 750;
        double stdev = cameraModel.getUncertainty(configuration, sampledPosition, targetHeight);
        double negTwoSigSquared = -2 * stdev * stdev;
        double probAtFourSig = Math.exp(16 * stdev * stdev / negTwoSigSquared);
        double fourSigSquareDist = (16 * stdev * stdev);
        for (int y = 0; y < locationProbabilities.length; y++) {
            double yPos = yOffset + binLength * y;
            double yDiff = yPos - sampledPosition[0];
            double[] arr = locationProbabilities[y];
            for (int x = 0; x < arr.length; x++) {
                double xPos = xOffset + binLength * x;
                if (zLow && (yLow != (yPos <= 0)) && (xLow != (xPos <= 0))) continue; //If the camera is below the origin and this point is in the opposite quadrant from the camera, the camera can't see the point.
                double xDiff = xPos - sampledPosition[1];
                double squareDist = xDiff * xDiff + yDiff * yDiff;
                //Use an approximate (and still low) value when outside of four sigma to speed up computation.
                arr[x] *= (squareDist < fourSigSquareDist) ?
                        Math.exp(squareDist / negTwoSigSquared) :
                        probAtFourSig;
            }
        }
        normalize();
    }
    
    //It's not exactly statistically rigorous, but it's reasonable.
    public void updateUnseen(ConfigurationCoordinate configuration, VisibleRange visibleRange) {
        WorkspaceCoordinate workspace = new WorkspaceCoordinate(configuration);
        boolean xLow = workspace.x <= 0;
        boolean yLow = workspace.y <= 0;
        boolean zLow = workspace.z <= 750;
        for (int y = 0; y < locationProbabilities.length; y++) {
            double yPos = yOffset + binLength * y;
            double[] arr = locationProbabilities[y];
            for (int x = 0; x < arr.length; x++) {
                double xPos = xOffset + binLength * x;
                if (zLow && (yLow != (yPos <= 0)) && (xLow != (xPos <= 0))) continue; //If the camera is below the origin and this point is in the opposite quadrant from the camera, the camera can't see the point.
                if (visibleRange.contains(yOffset + binLength * y, xOffset + binLength * x)) arr[x] /= 15;
            }
        }
        normalize();
    }
    
    public void normalize() {
        double sum = 0;
        for (int y = 0; y < locationProbabilities.length; y++) {
            for (int x = 0; x < locationProbabilities[y].length; x++) {
                sum += locationProbabilities[y][x];
            }
        }
        
        for (int y = 0; y < locationProbabilities.length; y++) {
            for (int x = 0; x < locationProbabilities[y].length; x++) {
                locationProbabilities[y][x] /= sum;
            }
        }
        dirty();
    }
    
    public void dirty() {
        this.cachedStdev = Double.NaN;
    }
    
    public double[] getHighestPosition() {
        int bestX = 0;
        int bestY = 0;
        for (int y = 0; y < locationProbabilities.length; y++) {
            for (int x = 0; x < locationProbabilities[y].length; x++) {
                if (locationProbabilities[y][x] > locationProbabilities[bestY][bestX]) {
                    bestY = y;
                    bestX = x;
                }
            }
        }
        return new double[] {yOffset + binLength * bestY, xOffset + binLength * bestX};
    }
    
    //Assumes a rectangular / non-jagged array
    public int getStateCount() {
        if (locationProbabilities.length == 0) return 0;
        return locationProbabilities.length * locationProbabilities[0].length;
    }
}
