package cv;

import geometry.Plane;
import geometry.PlaneDivider2;
import geometry.Quaternion;
import geometry.Ray;
import geometry.Vector3;

import commands.AbstractCoordinate;
import commands.ConfigurationCoordinate;
import commands.WorkspaceCoordinate;

public class CameraModel {
    static final int LIMB_LENGTH = 2500;
    static final int HAND_LENGTH = 1300;
    
    static final double X_FOV_SLOPE = 0.4d;
    static final double Y_FOV_SLOPE = 0.3d;

    static final int IMAGE_WIDTH = 640;
    static final int IMAGE_HEIGHT = 480;
    
    static final double CAMERA_UNCERTAINTY_RATIO = 0.05; //The (unitless) measure of the error in the camera-based detection. Conceptually, the units are cm/cm, and this number represents the standard deviation of the distance of the observed position of an object from its true position, divided by the distance of the camera from the object. (As the camera gets farther away from the object, the uncertainty increases linearly.)
    
    public VisibleRange getVisibleRange(ConfigurationCoordinate configuration, double planeHeight) {
        Vector3 position = getCameraLocation(configuration);
        Quaternion orientation = getCameraOrientation(configuration);
        
        Vector3[] originalCameraDirections = new Vector3[] { //The ray vectors corresponding to the corners of an image taken from the HOME position, starting in the upper-left and working clockwise
                new Vector3(-X_FOV_SLOPE, -Y_FOV_SLOPE, 1),
                new Vector3(+X_FOV_SLOPE, -Y_FOV_SLOPE, 1),
                new Vector3(+X_FOV_SLOPE, +Y_FOV_SLOPE, 1),
                new Vector3(-X_FOV_SLOPE, +Y_FOV_SLOPE, 1),
        };
        
        Quaternion[] originalCameraRayQuats = new Quaternion[originalCameraDirections.length];
        for (int x = 0; x < originalCameraRayQuats.length; x++) originalCameraRayQuats[x] = Quaternion.fromFromTo(Vector3.UP, originalCameraDirections[x]);
        
        Quaternion[] finalCameraQuats = new Quaternion[originalCameraRayQuats.length];
        for (int x = 0; x < finalCameraQuats.length; x++) finalCameraQuats[x] = orientation.mult(originalCameraRayQuats[x]);
        
        Ray[] cameraRays = new Ray[finalCameraQuats.length];
        for (int x = 0; x < cameraRays.length; x++) cameraRays[x] = new Ray(position, Vector3.UP.afterRotation(finalCameraQuats[x]));
        
        Plane targetPlane = new Plane(new Vector3(0, 0, planeHeight), Vector3.UP);
        
        PlaneDivider2[] planeDividers = new PlaneDivider2[cameraRays.length];
        for (int x = 0; x < planeDividers.length; x++) planeDividers[x] = PlaneDivider2.fromRayPair(cameraRays[x], cameraRays[(x + 1) % planeDividers.length], targetPlane);
        
        return new VisibleRange(planeDividers);
    }
    
    public double getUncertainty(ConfigurationCoordinate configuration, double[] position, double targetHeight) {
        WorkspaceCoordinate workspace = new WorkspaceCoordinate(configuration);
        Vector3 wristEndpoint = new Vector3(workspace.x, workspace.y, workspace.z);
        Quaternion cameraOrientation = getCameraOrientation(configuration);
        Vector3 cameraEndpoint = wristEndpoint.add(Vector3.UP.afterRotation(cameraOrientation).withScale(HAND_LENGTH));
        Vector3 targetVector = new Vector3(position[1], position[0], targetHeight);
        double distance = cameraEndpoint.distanceTo(targetVector);
        Vector3 camToTarg = targetVector.sub(cameraEndpoint).normalized();
        Vector3 camDir = Vector3.UP.afterRotation(cameraOrientation).normalized();
        double cos = camDir.dot(camToTarg);
        double sin = Math.sin(Math.acos(cos));
        double slope = Math.abs(sin/cos);
        return distance * CAMERA_UNCERTAINTY_RATIO * (1 + slope); //Trust the camera less out of the corner of its eye.
    }
    
    public Vector3 getWristLocation(ConfigurationCoordinate configuration) {
        WorkspaceCoordinate workspace = new WorkspaceCoordinate(configuration);
        return new Vector3(workspace.x, workspace.y, workspace.z);
    }
    
    public Quaternion getCameraOrientation(ConfigurationCoordinate configuration) {
        return Quaternion.fromAxisAngle(Vector3.DOWN, AbstractCoordinate.roboToRadians(configuration.waist)).mult(
               Quaternion.fromAxisAngle(Vector3.LEFT, AbstractCoordinate.roboToRadians(configuration.shoulder))).mult(
               Quaternion.fromAxisAngle(Vector3.LEFT, AbstractCoordinate.roboToRadians(configuration.elbow))).mult(
               Quaternion.fromAxisAngle(Vector3.LEFT, AbstractCoordinate.roboToRadians(configuration.wrist))).mult(
               Quaternion.fromAxisAngle(Vector3.UP, AbstractCoordinate.roboToRadians(configuration.hand)));
    }
    
    public Vector3 getCameraLocation(ConfigurationCoordinate configuration) {
        Vector3 wristLocation = getWristLocation(configuration);
        Quaternion cameraOrientation = getCameraOrientation(configuration);
        return wristLocation.add(Vector3.UP.afterRotation(cameraOrientation).withScale(HAND_LENGTH));
    }
    
    public double[] mapToTarget(double[] imageCoord, ConfigurationCoordinate configuration, double targetHeight) {
        int halfWidth = IMAGE_WIDTH / 2;
        int halfHeight = IMAGE_HEIGHT / 2;
        double[] scaled = new double[] {((imageCoord[0] + 0.5) - halfHeight) / halfHeight, ((imageCoord[1] + 0.5) - halfWidth) / halfWidth};
        Vector3 unAlignedDirection = new Vector3(scaled[1] * X_FOV_SLOPE, scaled[0] * Y_FOV_SLOPE, 1);
        Quaternion rayOrientation = getCameraOrientation(configuration).mult(Quaternion.fromFromTo(Vector3.UP, unAlignedDirection));
        Ray ray = new Ray(getCameraLocation(configuration), Vector3.UP.afterRotation(rayOrientation));
        Vector3 intersectionPoint = ray.intersectionPoint(new Plane(new Vector3(0, 0, targetHeight), new Vector3(0, 0, 1)));
        if (intersectionPoint == null) return null;
        else return new double[] {intersectionPoint.y, intersectionPoint.x, imageCoord[2]};
    }
}
