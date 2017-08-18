package cv;

import georegression.metric.UtilAngle;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;

import javax.imageio.ImageIO;

import boofcv.alg.color.ColorHsv;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.Planar;

public class BlobIdentifier {
    
    //The image segmentation of this method comes mostly from the BoofCV color segmentation example
    //Returns the centerpoints of each blob, of the form [YCenter, XCenter, BlobSize], sorted by blob size (largest to smallest)
    public static List<double[]> getCenterLocations(BufferedImage image, float targetHue, float targetSaturation, float squareMaxDist, int minBlobArea) {
        Planar<GrayF32> input = ConvertBufferedImage.convertFromMulti(image,null,true,GrayF32.class);
        Planar<GrayF32> hsv = input.createSameShape();
 
        // Convert into HSV
        ColorHsv.rgbToHsv_F32(input,hsv);
 
        // Extract hue and saturation bands which are independent of intensity
        GrayF32 H = hsv.getBand(0);
        GrayF32 S = hsv.getBand(1);
 
        // Adjust the relative importance of Hue and Saturation.
        // Hue has a range of 0 to 2*PI and Saturation from 0 to 1.
        float adjustUnits = (float)(Math.PI/2.0);
 
        // step through each pixel and mark how close it is to the selected color
        boolean[][] isPresent = new boolean[hsv.height][hsv.width];
        for( int y = 0; y < hsv.height; y++ ) {
            for( int x = 0; x < hsv.width; x++ ) {
                // Hue is an angle in radians, so simple subtraction doesn't work
                float dh = UtilAngle.dist(H.unsafe_get(x,y),targetHue);
                float ds = (S.unsafe_get(x,y)-targetSaturation)*adjustUnits;
 
                // this distance measure is a bit naive, but good enough for to demonstrate the concept
                float dist2 = dh*dh + ds*ds;
                if( dist2 <= squareMaxDist ) {
                    isPresent[y][x] = true;
                }
            }
        }
        return getCenters(isPresent, minBlobArea);
    }
    
    static List<double[]> getCenters(boolean[][] isPresent, int minBlobArea) {
        List<double[]> centers = new LinkedList<double[]>();
        for (int y = 0; y < isPresent.length; y++) {
            for (int x = 0; x < isPresent[y].length; x++) {
                List<int[]> filledSquares = new LinkedList<int[]>();
                stacklessFloodFill(isPresent, y, x, filledSquares);
                if (filledSquares.size() >= minBlobArea) centers.add(getCenter(filledSquares));
            }
        }
        Collections.sort(centers, Comparator.<double[]>comparingDouble(dA -> dA[2]).reversed());
        return centers;
    }
    
    static double[] getCenter(List<int[]> points) {
        if (points.isEmpty()) return null;
        double[] sum = new double[points.get(0).length];
        for (int[] point : points) for (int x = 0; x < point.length; x++) sum[x] += point[x];
        double[] averageAndSize = new double[sum.length + 1];
        for (int x = 0; x < sum.length; x++) averageAndSize[x] = sum[x] / points.size();
        averageAndSize[sum.length] = points.size();
        return averageAndSize;
    }
    
    //adds the [y,x] coordinates of all tiles filled in to the provided filledSquares list
    //is destructive on the input isPresent array
    //transitioned from the traditional recursive floodFill due to stack overflow errors.
    static void stacklessFloodFill(boolean[][] isPresent, int yStart, int xStart, List<int[]> filledSquares) {
        List<int[]> toVisit = new LinkedList<int[]>();
        toVisit.add(new int[] {yStart, xStart});
        while (!toVisit.isEmpty()) {
            int[] point = toVisit.remove(0);
            int y = point[0];
            int x = point[1];
            if (y < 0 || y >= isPresent.length || x < 0 || x >= isPresent[0].length || !isPresent[y][x]) continue;
            isPresent[y][x] = false;
            filledSquares.add(new int[] {y, x});
            for (int dX = -1; dX <= 1; dX++) {
                for (int dY = -1; dY <= 1; dY++) {
                    if (dX != 0 || dY != 0) {
                        toVisit.add(new int[] {y + dY, x + dX});
                    }
                }
            }
        }
    }
    
    public static List<double[]> getCenterLocations(BufferedImage image) {
        return getCenterLocations(image, 6f, 0.65f, 0.4f*0.4f, 50);
    }
    
    public static List<double[]> getCenterLocations(String fileName) throws IOException {
        return getCenterLocations(ImageIO.read(new File(fileName)));
    }
    
    public static void main (String[] args) throws IOException {
        String fileName = "Pipe - Resized.png";
        for (double[] center : getCenterLocations(fileName)) {
            System.out.printf("x: %f y: %f%n", center[1], center[0]);
        }
    }
}
