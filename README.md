# Team 1672 CHARGED UP

The codebase which we used for the 2023 FIRST Robotics Competition Game, CHARGED UP. 

## Branches
| Main | Auto | Simulation |
|------|------|------------|
| The main code on our robot used at competition.      |  Code which has SysID constants and Pathplanner to perform an auto routine. *Currently not working*    | Contains the variables from the SysID testing to bring simulation to the Robot. *Currently only supports Drivebase simulation*|

## PhotonVision
Our robot uses a [Beelink Mini PC](https://www.bee-link.com/catalog/product/buy?id=303) running PhotonVision to be able to detect AprilTags. We then use this for approaching the double substation, so the driver does not have to guess the distance. 

## Operator Controller
Our team uses a keypad which we got off of Amazon to be able to control the scoring functionality of the Robot. We use a program called [KeyboardSplitter](https://github.com/djlastnight/KeyboardSplitterXbox) to be able to map the keyboard to an Xbox Controller which we then use for inside of our codebase. 

##  RGB Lighting
The RGB lighting is using AndyMark Addressable RGB lighting, and changes based on the state of the robot. 


**Disabled** - *Pulsating based on our alliance color*
**Enabled** - *Rainbow*
**Operator Controller** - *Can change it to be yellow or purple based on what game piece we want* 
