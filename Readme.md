## User manual for tools for scanning objects with several L515 Lidars

The programs will allow you to scan objects or read previously scanned files.

In order to enable scanning, we use the: --lidar option

To load previously scanned files, use the option: --input_folder = <directory where files with ply extension are located>. For example --input_folder = "Kasia/1"

Objects are scanned as described https://dev.intelrealsense.com/docs/lidar-camera-l515-multi-camera-setup. Individual cameras are controlled by means of the FT232H system connected to the same station as the L515.

Two tools are available:
1. Main scanning tool - main.py
2. Auxiliary tool for automatic calculation of isomorphic spatial transformations of individual point clouds in relation to the camera coordinate system - auto_transform_calculator.py

###### Connecting lidars with GPIO FT232H ports
FT232H configuration: https://learn.adafruit.com/circuitpython-on-any-computer-with-ft232h/linux

Both tools require the environment variable to be entered as: BLINKA_FT232H = 1
In the `scene_reader.py` file there is a dictionary definition consisting of the last three digits of the L515 serial number and the associated GPIO ports:

`self.gpio_config = {
             '303': board.C1,
             '608': board.C3,
             '337': board.C2,
             '630': board.C0
         } `

   In case of a different connection, change the above configuration.


### Tool for automatic calculation of transformations

1. Running the tool: `python3 auto_transform_calculator.py`
2. The tool automatically detects the number of connected lidars.
3. After reading the data from the lidars, the windows open in pairs: SOURCE, TARGET. The tool will look for a SOURCE transformation to match TARGET. The first connected lidar becomes the first TARGET.
4. Mark correspondence points at point cloud TARGET and SOURCE. Minimum 3. Selection / deselection is performed by SHIFT + left / right mouse button.
5. If we are sure about the selection, we close the windows with the `q` key.
6. A preview window appears on what SOURCE looks like in relation to TARGET after the transformation. We close the window with `q`.
7. If there are more lidars, another pair of TARGET and SOURCE windows will appear, but this time TARGET will contain the previous two point clouds after the last transformation.
8. We continue the procedure until all point clouds from all lidars are processed.
9. As a result, the program generates a file named `auto_transform_matrices.npy`, which is automatically loaded by the main scanning utility.

### Tool for automatic calculation of transformations

1. Running the tool: `python3 main.py`
2. The tool automatically loads the `auto_transform_matrices.npy` file, which allows you to pre-transform the data from individual lidars.
3. Then the individual point clouds from the lidars are read sequentially.
4. All read point clouds are displayed in one window. Transformations on individual point clouds are performed by indicating which point could be active using the `F1, F2, F3, F4` keys. Initially, all read point clouds are active. The `F12` key activates all point clouds.
5. List of active point clouds transformations:
    * `A <-> D` shift left / right X axis
    * `W <-> S` Offset left / right Y axis
    * `Z <-> X` shift left / right Z axis
    * `R <-> T` rotation left / right X axis
    * `F <-> G` rotation left / right Y axis
    * `V <-> B` rotation left / right Z axis
    * `Backspace` performs a back transform
    * `P` display isomorphic transformation matrix
    * `O` notation of isomorphic transformation matrix
    * `L` reads the isomorphic transformation matrix
    * `I` export ALL point ploud in one PLY file
    * `.` voxel down sample
    * `,` radius filter
    * `M` creating a mesh
    * `/` change display geometry pount coulds / mesh
    * `U` ​​export the mesh to an OBJ file
    * `E` reset camera position
    * `C` normals estimate for active point clouds. NOTE: the position of the camera affects which way the normals vectors will be pointed. The direction of the vectors affects the meshing process.
6. Close the window and exit the `q` program.
7. List of other default `h` commands

    * [Open3D INFO]   -- Mouse view control --
    * [Open3D INFO]     Left button + drag         : Rotate.
    * [Open3D INFO]     Ctrl + left button + drag  : Translate.
    * [Open3D INFO]     Wheel button + drag        : Translate.
    * [Open3D INFO]     Shift + left button + drag : Roll.
    * [Open3D INFO]     Wheel                      : Zoom in/out.
 
    * [Open3D INFO]   -- Keyboard view control --
    * [Open3D INFO]     [/]          : Increase/decrease field of view.
    * [Open3D INFO]     R            : Reset view point.
    * [Open3D INFO]     Ctrl/Cmd + C : Copy current view status into the clipboard.
    * [Open3D INFO]     Ctrl/Cmd + V : Paste view status from clipboard.
 
    * [Open3D INFO]   -- General control --
    * [Open3D INFO]     Q, Esc       : Exit window.
    * [Open3D INFO]     H            : Print help message.
    * [Open3D INFO]     P, PrtScn    : Take a screen capture.
    * [Open3D INFO]     D            : Take a depth capture.
    * [Open3D INFO]     O            : Take a capture of current rendering settings.
    * [Open3D INFO]     Alt + Enter  : Toggle between full screen and windowed mode.
 
    * [Open3D INFO]   -- Render mode control --
    * [Open3D INFO]     L            : Turn on/off lighting.
    * [Open3D INFO]     +/-          : Increase/decrease point size.
    * [Open3D INFO]     Ctrl + +/-   : Increase/decrease width of geometry::LineSet.
    * [Open3D INFO]     N            : Turn on/off point cloud normal rendering.
    * [Open3D INFO]     S            : Toggle between mesh flat shading and smooth shading.
    * [Open3D INFO]     W            : Turn on/off mesh wireframe.
    * [Open3D INFO]     B            : Turn on/off back face rendering.
    * [Open3D INFO]     I            : Turn on/off image zoom in interpolation.
    * [Open3D INFO]     T            : Toggle among image render:
    * [Open3D INFO]                    no stretch / keep ratio / freely stretch.

    * [Open3D INFO]   -- Color control --
    * [Open3D INFO]     0..4,9       : Set point cloud color option.
    * [Open3D INFO]                    0 - Default behavior, render point color.,00
    * [Open3D INFO]                    1 - Render point color.
    * [Open3D INFO]                    2 - x coordinate as color.
    * [Open3D INFO]                    3 - y coordinate as color.
    * [Open3D INFO]                    4 - z coordinate as color.
    * [Open3D INFO]                    9 - normal as color.
    * [Open3D INFO]     Ctrl + 0..4,9: Set mesh color option.
    * [Open3D INFO]                    0 - Default behavior, render uniform gray color.
    * [Open3D INFO]                    1 - Render point color.
    * [Open3D INFO]                    2 - x coordinate as color.
    * [Open3D INFO]                    3 - y coordinate as color.
    * [Open3D INFO]                    4 - z coordinate as color.
    * [Open3D INFO]                    9 - normal as color.
    * [Open3D INFO]     Shift + 0..4 : Color map options.
    * [Open3D INFO]                    0 - Gray scale color.
    * [Open3D INFO]                    1 - JET color map.
    * [Open3D INFO]                    2 - SUMMER color map.
    * [Open3D INFO]                    3 - WINTER color map.
    * [Open3D INFO]                    4 - HOT color map.
     
    
.