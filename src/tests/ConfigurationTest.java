package tests;

import commands.ConfigurationCoordinate;
import commands.WorkspaceCoordinate;

public class ConfigurationTest {
    
    /*
     * Limitations in the transmitted space: (TODO: convert these into true configuration space)
     * 
     * Waist: [-18000,18000] //But notice that this is actually limited further, because of the conversion coefficient.
     * Shoulder: [-11750, 11750]
     * Elbow: [-13500, 13500]
     * Wrist: [-10125, 10125]
     * Hand Twist: [-18000, 18000]
     * 
     * X, Y: [-4800, 4800]     [-2200, 2200] restricted for negative Z
     * z: [-1800, 5000]
     */
    
    public static void main (String[] args) {
        //System.out.println(new ConfigurationCoordinate(-9000, 0, 9000, 9000, 0).getUnescapedCommandString());
        System.out.println(new ConfigurationCoordinate(new WorkspaceCoordinate(3850, -2050, -1500, 0, 0)).getUnescapedCommandString());
    }
}
