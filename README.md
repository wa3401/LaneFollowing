# LaneFollowing

**Will Anderson**

### This repo contains a full package for autonomus lane following

## Image processing
### Taken care of fully by the ImageProcessor header file
### Steps for image processing
   1. Import `imageProcessor.h`
   2. Create an `ImageProcessor` object
   3. Define mins and maxes for Hue-Saturation-Value (See HSV Value Finding section below)
   4. Call `getBlur(Scalar min, Scalar max, Mat inputImage)`
       * This returns a blurred CV Mat that is color filtered to your mins and maxes
   5. Call `getErode(blurredImage)` using output of last function
       * This returns an erroded CV Mat which you will later publish to a window so you can see the lines in question
   6. Define a `vector<vector<double>> *color*LaneLines` (*color* being whatever color you have filtered out)
   7. Set that vector equal to a call of `processImage(Mat erodeImage, Mat bluredImage)`
       * This returns a `<vector<vector<double>>`
       * The inner vector holds **slope** and **x coordinate** of each line found
          * Access the x coordinate using `*color*LaneLines.at(idx).at(0)`
          * Access the slope using `*color*LaneLines.at(idx).at(1)`
       * The outer vector holds however many lines of that color are found
       
## Finding HSV Values **STEPS**

**Default Values Are As Follows**

           `//White mins and maxes for Hue-Saturation-Value model
            int whmin = 34, whmax = 179, wsmin = 0, wsmax = 255, wvmin = 252, wvmax = 255;
            
            //Yellow mins and maxes for Hue-Saturation-Value model
            int yhmin = 19, yhmax = 49, ysmin = 34, ysmax = 126, yvmin = 175, yvmax = 255;`

  1. Change sensortocv exectuteble in `CMakeLists.txt` file to rosImageToCV.cpp
  2. Launch `sensortocv.launch`
      * Trackbars will show up, move the mins and maxes so only the color you want shows up in white in the view window
  3. Take a picture of the values
  4. Define them in your file as
  
  `int hmin = 34, hmax = 179, smin = 0, smax = 255, vmin = 252, vmax = 255;`
     
  `Scalar Lower(hmin, smin, vmin);`
  
  `Scalar wUpper(hmax, smax, vmax);`
  
  5. Plug the scalars in to the `getBlur` function from the `imageProcessor.h` header
  
  
## Calculating Steering Angle
  1. Using above steps, you now have `vector<vector<double>>` conataining lines with slopes and x intercepts
  2. Use these values however you see fit to calculate steering angle
  
## Helpful Tips
- Use Hyperbpolic tangent function (`tanh(x)`) * `MAX_STEERING_ANGLE` for so that your angle is never greater than the max
- Camera is not centered on the car, meaning middle of the image should not be centered in the lane
  * You can see how I took care of this using 
  
    `double difInSlope = abs(wSlope) - abs(ySlope);`
    
    `double avgSlope = (wSlope + ySlope) / 2 + difInSlope / 2;`
  
