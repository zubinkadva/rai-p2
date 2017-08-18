package visualization;

import georegression.metric.UtilAngle;

import java.awt.image.BufferedImage;

import boofcv.alg.color.ColorHsv;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.Planar;

public class SegmentVisualizer {
    public static void showSegmentationWindow(String title, BufferedImage image, float hue, float saturation) {
        Planar<GrayF32> input = ConvertBufferedImage.convertFromMulti(image,null,true,GrayF32.class);
        Planar<GrayF32> hsv = input.createSameShape();
 
        // Convert into HSV
        ColorHsv.rgbToHsv_F32(input,hsv);
 
        // Euclidean distance squared threshold for deciding which pixels are members of the selected set
        float maxDist2 = 0.4f*0.4f;
 
        // Extract hue and saturation bands which are independent of intensity
        GrayF32 H = hsv.getBand(0);
        GrayF32 S = hsv.getBand(1);
 
        // Adjust the relative importance of Hue and Saturation.
        // Hue has a range of 0 to 2*PI and Saturation from 0 to 1.
        float adjustUnits = (float)(Math.PI/2.0);
 
        // step through each pixel and mark how close it is to the selected color
        BufferedImage output = new BufferedImage(input.width,input.height,BufferedImage.TYPE_INT_RGB);
        for( int y = 0; y < hsv.height; y++ ) {
            for( int x = 0; x < hsv.width; x++ ) {
                // Hue is an angle in radians, so simple subtraction doesn't work
                float dh = UtilAngle.dist(H.unsafe_get(x,y),hue);
                float ds = (S.unsafe_get(x,y)-saturation)*adjustUnits;
 
                // this distance measure is a bit naive, but good enough for to demonstrate the concept
                float dist2 = dh*dh + ds*ds;
                if( dist2 <= maxDist2 ) {
                    output.setRGB(x,y,image.getRGB(x,y));
                }
            }
        }
 
        ShowImages.showWindow(output, title);
        UtilImageIO.saveImage(output, "savedVisualizations/" + title + ".bmp");
    }
    
    public static void displaySegmentation(String title, BufferedImage image) {
        ShowImages.showWindow(image,title + " Unsegmented");
        UtilImageIO.saveImage(image, "savedVisualizations/" + title + " Unsegmented.bmp");
        showSegmentationWindow(title + " Segmented", image, 6.0f,0.65f);
    }
    
    public static void displaySegmentation(String title, String fileName) {
        displaySegmentation(title, UtilImageIO.loadImage(fileName));
    }
}
