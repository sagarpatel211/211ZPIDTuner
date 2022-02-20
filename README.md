# 211ZPIDTuner
This is our PID-controller tuner graph in VexCode that allows Team 211 to visualize the values of the motor to see if the PID controller is working correctly and is tuned perfectly. With slight changes, it can be used to graphically see the plotting of any desired value and any actual value you desire!


## Table of Contents
* [Dependencies](#dependencies)
* [Installation](#installation)
* [Features](#features)
* [Contributors](#contributors)
* [Contact](#contact)


## Dependencies
[VexCode Pro V5 Text 2.0.0 or later](https://www.vexrobotics.com/vexcode-download)


## Installation
* Make sure all the dependencies are installed
* Download the files
  * Option 1: 🍴 Fork this repository!
  * Option 2: 🧪 Clone the repository to your local machine using https://github.com/sagarpatel211/211ZPIDTuner.git!
* Open *211Z_PIDTuner.v5code* in VexCode to open the program
* Download the program to the brain by connecting the V5 Brain or controller to the device via micro-USB and select *download*. In both options, the V5 Brain must be on!
* Run the program by selecting it from the V5 Brain or pressing the *play* button in VexCode **if** the V5 Brain or controller is attached to the device via micro-USB.


## Features
* V5 Brain screen displays current motor rotataional value and target motor value as lines on a graph
* Customizable to as many graphed lines as you need
* Our example (change the value of the *motordegree* and *Columndesired* to change the value being plotted on the graph):
```cpp
int ActualValueTask( void *arg ) {
    if( arg == NULL ) return 0;
    graph *g = static_cast<graph *>(arg);
    int motordegree = ((PIDMotor.rotation(rotationUnits::deg)/4)); <- Change value (actual; green line)!
    while(1) {
      g->addPoint( 0, motordegree);
      this_thread::sleep_for(15);
    }
}
int TargetValueTask( void *arg ) {
    if( arg == NULL ) return 0;
    graph *g = static_cast<graph *>(arg);
    while(1) {
      g->addPoint( 1, ColumnDesired/4); <- Change ColumnDesired (desired; red line)!
      this_thread::sleep_for(15);
    }
}
```
![Example](https://github.com/sagarpatel211/211ZPIDTuner/blob/master/Example%20PID%20Graph%20Image.png)


## Contributors
| <a href="https://github.com/sagarpatel211" target="_blank">**Sagar Patel**</a> |
| :---: |
| [![Sagar Patel](https://avatars1.githubusercontent.com/u/34544263?s=200)](https://github.com/sagarpatel211)    |
| <a href="https://github.com/sagarpatel211" target="_blank">`github.com/sagarpatel211`</a> |


## Contact
[Email](mailto:sa24pate@uwaterloo.ca) | [Website](https://sagarpatel211.github.io/)
