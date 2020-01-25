# SpeechRecognition
OpenRobotics development for RoboCup @ Home Education SPeech recognition and synthesis tasks

## Setting up the Workspace
SpeechRecognition is not a workspace, it is a package, so it requires a workspace.
To setup a catkin workspace, 
`mkdir -p ~/catkin_ws/src
cd catkin_ws
catkin_make`

## Cloning and Catkin_make
After setting up a workspace, run

`cd ~/catkin_ws/src
git clone https://github.com/UBC-OpenRobotics/SpeechRecognition.git
`
Then,

`cd ~/catkin_ws
catkin_make`



## Installing Pocketsphinx
(add slides commands)
If PocketSphinx isn't installing correctly, try running:
`pip install --upgrade setuptools 
sudo apt-get install libpulse-dev
`
## Getting the language model

You can download the language model from [here](https://sourceforge.net/projects/cmusphinx/files/Acoustic%20and%20Language%20Models/Archive/US%20English%20HUB4WSJ%20Acoustic%20Model/)

Then you need to place its contents in `/usr/local/share/pocketsphinx/model/en-us/en-us`

