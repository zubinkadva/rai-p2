package commands;
public class ConfigurationCoordinate extends AbstractCoordinate {
    //These variables are expressed in robot units, where:
    //1 cm = 100 robot units, and
    //1 degree = 100 robot units
    public final double waist, shoulder, elbow, wrist, hand;
    
    //The camera is not anchored exactly on the robot, so this offset is required to level it.
    public static final int HAND_OFFSET = -5550; //TODO: Calibrate this again.
    public static final double WAIST_MULTIPLE = 10500d / 9000d;
    
    public ConfigurationCoordinate(double waist, double shoulder, double elbow, double wrist, double hand) {
        this.waist = waist;
        this.shoulder = shoulder;
        this.elbow = elbow;
        this.wrist = wrist;
        this.hand = hand;
    }
    
    //Copy constructor
    public ConfigurationCoordinate(ConfigurationCoordinate o) {
        this(o.waist, o.shoulder, o.elbow, o.wrist, o.hand);
    }
    
    //Inverse kinematics constructor
    public ConfigurationCoordinate(WorkspaceCoordinate o) {
        double waist = Math.atan2(o.x, o.y);
        double horDist = Math.sqrt(o.x * o.x + o.y * o.y);
        double dist = Math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z);
        double z = o.z;
        double centerAngle = Math.acos((2 * LIMB_LENGTH * LIMB_LENGTH - dist * dist) / (2 * LIMB_LENGTH * LIMB_LENGTH));
        double elbow = Math.PI - centerAngle;
        double vertToWrist = Math.atan2(horDist, z);
        double elbToWrist = Math.asin((LIMB_LENGTH / dist) * Math.sin(centerAngle));
        double shoulder = vertToWrist - elbToWrist;
        double wristPointAngle = shoulder + elbow;
        double desiredWrist = (Math.PI / 2) - roboToRadians(o.wrist);
        double wrist = desiredWrist - wristPointAngle;
        double hand = o.handTwist;
        
        //Because of the waist multiple, the robot doesn't quite behave as expected around 180 degrees waist twist with the above configurations set.
        //Translate the configuration to constrain the waist to (-90, 90], and allow the shoulder to bend backwards, rather than letting the waist move (-180, 180] and constraining the shoulder to only bend forward.
        if (waist <= -Math.PI / 2 || waist > Math.PI / 2) {
            waist = roboToRadians(normalize(radiansToRobo(waist) + 18000));
            shoulder = -shoulder;
            elbow = -elbow;
            wrist = -wrist;
            hand = roboToRadians(normalize(radiansToRobo(waist) + 18000));
        }
        
        //Unfortunately, I can't call a constructor down here.
        this.waist = radiansToRobo(waist);
        this.shoulder = radiansToRobo(shoulder);
        this.elbow = radiansToRobo(elbow);
        this.wrist = radiansToRobo(wrist);
        this.hand = hand;
    }
    
    @Override
    public String getUnescapedCommandString() {
        return String.format("%d %d %d %d %d AJMA", normalize(hand + HAND_OFFSET), normalize(wrist), normalize(elbow), normalize(shoulder), normalize(waist * WAIST_MULTIPLE));
    }
    
    //TODO: Add a function to check if the configuration is valid according to Dr. Silaghi's constraints.
}
