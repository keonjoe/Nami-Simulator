# Nami-Simulator
A MBD and Beam based simulator based on the Project Chrono physics engine. The goal of this project is to build a tool to better characterize the loads on the main steering shaft of the Nami Burn E scooter and better understand the catastrophic failure mode in which the shaft can fracture.
![image](https://user-images.githubusercontent.com/22308960/221742415-0f9a55ee-e3bf-40af-bb90-71821a18af2e.png)
![vonMises](https://user-images.githubusercontent.com/22308960/228398093-f02f119c-5697-4c76-8dc9-5a86590eb947.png)

## How to compile:
To compile, you will need to follow the installation instructions for Project Chrono: https://api.projectchrono.org/tutorial_install_chrono.html. Once complete, you will need to configure this project in CMake and copy+paste the data/ directory from the Project Chrono install into the build folder for this project.

## Using the precompiled binaries:
To test the tool out using the precompiled binaries, please follow these steps:
1. Create a folder structure like the following:
```
.Root
├── data                     # Contains common visual elements for display window
└──  vx.x                    # Contains executable files of the simulator
     └──  nami_shapes        # Contains .obj files for visual geometry
```
2. Clone this repository and place the contents of 'data/' into 'Root/data/'. Contents of 'nami_shapes' will need to placed in 'Root/vx.x/nami_shapes'.
3. Download the [Intel MKL Library](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onemkl.html) and follow the unofficial method of setting the environment variable section under "Set up the environment" of this guide: https://api.projectchrono.org/development/module_mkl_installation.html
4. Download the precompiled binaries under [Releases](https://github.com/keonjoe/Nami-Simulator/releases). Place these in vx.x, with x.x matching the release version you download.
5. Run the model by launching from command line with the appropriate command line arguments, or by using the nami_stem.bat batch script. The nami_stem.bat script included with each release contains comments relating to what the various command line arguments do.

## Simulation data (simout.zip)
I have added a .zip directory containing simulation results created using release 0.0 of this program for a wide range of parameters. You can look at the parameters that have been swept by investigating the values in the .bat files in this repository. 

Once extracted, the results directory will have the following structure:
```
simout
├── beamwwkg_xxkph_shape_yyhxzzw     # Folder containing simulation results for a rider weighing <ww> kg, traveling at <xx> kph, over a 
└── beamwwkg_xxkph_shape_yyhxzzw     # bump of <shape> type that has a height of <yy> meters perpendicular to the ground and extends <zz> meters horizontally
     ├── frames              # Contains frames exported from the simulation. Empty if no animations were saved or visualization was disabled for the run
     ├── columnForces.csv    # Contains the x,y,z forces and moments for the constraint connecting the steerer and column, as well as the max stress in the column
     ├── mz.dat              # Data file for the moment about the z axis for the constraint connecting the steerer and column
     ├── mz.png              # Plot of mz.dat
     ├── vonMises.dat        # Data file for the maximum stress in the steering column 
     └── vonMises.png        # Plot of vonMimses.png 
```
For more information about the possible parameters and the valid ranges, please consult the comments about the program input arguments in nami_stem.cpp

## Some more context on the issue:
It is suspected that the AL 7075 steering shaft present in all Nami Burn E scooters manufactured before Jan. 2023 are seeing stress from normal use which eventually exceed the typical fatigue limit for aluminum 7075. Several scooters have failed suddenly and catastrophically due to this suspected design oversight.

Anecdotal cases can be found in the following Facebook post: https://www.facebook.com/groups/namiworldwide/posts/1246539072876878/

![image](https://user-images.githubusercontent.com/22308960/221741353-53e77080-0c12-4dd1-ac0b-46c8455093dc.png)
![image](https://user-images.githubusercontent.com/22308960/221741426-128e19f0-db9a-4280-bdd8-969bab90e73c.png)
![image](https://user-images.githubusercontent.com/22308960/221741478-13a3bd77-2006-4a0d-b857-c24d228d77ff.png)
![image](https://user-images.githubusercontent.com/22308960/221741614-674bdb01-ae13-430f-92e4-e4d6043cdf7d.png)
![image](https://user-images.githubusercontent.com/22308960/221741637-70b9401b-679f-467f-8c47-68b469e35f55.png)
![image](https://user-images.githubusercontent.com/22308960/221741660-cd73e399-22a5-4a98-9c8c-6a3a4331c4d7.png)
