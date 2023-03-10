# Nami-Simulator
A MBD and Beam based simulator based on the Project Chrono physics engine. The goal of this project is to build a tool to better characterize the loads on the main steering shaft of the Nami Burn E scooter and better understand the catastrophic failure mode in which the shaft can fracture.
![image](https://user-images.githubusercontent.com/22308960/221742415-0f9a55ee-e3bf-40af-bb90-71821a18af2e.png)
![image](https://user-images.githubusercontent.com/22308960/221742497-f40b7ab3-83c5-49ed-9d01-54a9d43c6913.png)

To compile, you will need to follow the installation instructions for Project Chrono: https://api.projectchrono.org/tutorial_install_chrono.html. Once complete, you will need to configure this project in CMake and copy+paste the data/ directory from the Project Chrono install into the build folder for this project.

Once the project is more mature, I will add precompiled versions along with more user friendly UI features.

Some more context on the issue:
It is suspected that the AL 7075 steering shaft present in all Nami Burn E scooters manufactured before Jan. 2023 are seeing stress from normal use which eventually exceed the typical fatigue limit for aluminum 7075. Several scooters have failed suddenly and catastrophically due to this suspected design oversight.

Anecdotal cases can be found in the following Facebook post: https://www.facebook.com/groups/namiworldwide/posts/1246539072876878/

![image](https://user-images.githubusercontent.com/22308960/221741353-53e77080-0c12-4dd1-ac0b-46c8455093dc.png)
![image](https://user-images.githubusercontent.com/22308960/221741426-128e19f0-db9a-4280-bdd8-969bab90e73c.png)
![image](https://user-images.githubusercontent.com/22308960/221741478-13a3bd77-2006-4a0d-b857-c24d228d77ff.png)
![image](https://user-images.githubusercontent.com/22308960/221741614-674bdb01-ae13-430f-92e4-e4d6043cdf7d.png)
![image](https://user-images.githubusercontent.com/22308960/221741637-70b9401b-679f-467f-8c47-68b469e35f55.png)
![image](https://user-images.githubusercontent.com/22308960/221741660-cd73e399-22a5-4a98-9c8c-6a3a4331c4d7.png)
