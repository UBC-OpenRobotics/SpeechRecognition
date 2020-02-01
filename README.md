# SpeechRecognition
OpenRobotics development for RoboCup @ Home Education Speech recognition and Synthesis tasks

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
```
sudo apt-get install python-pip python-dev build-essential
sudo pip install --upgrade pip
sudo apt-get istall libasound-dev
sudo apt-get install python-pyaudio
sudo pip install pyaudio
sudo apt-get install swig
sudo pip install pocketsphinx
```

Then, to install the ROS package and wrapper,
```
cd ~/catkin_ws/src
git clone https://github.com/Pankaj-Baranwal/pocketsphinx
cd ~/catkin_ws
catkin_make
```


If PocketSphinx isn't installing correctly, try running:

```
pip install --upgrade setuptools
sudo apt-get install libpulse-dev
```
## Getting the language model

You can download the language model from [here](https://sourceforge.net/projects/cmusphinx/files/Acoustic%20and%20Language%20Models/Archive/US%20English%20HUB4WSJ%20Acoustic%20Model/)

Then you need to place its contents in `/usr/local/share/pocketsphinx/model/en-us/en-us` . Most likely, these folders dont exist, in which case you should just create them `cd /usr/local/share ; sudo mkdir -p pocketsphinx/model/en-us/en-us`

You can extract the tarball using `sudo tar -xzvf hub4wsj_sc_8k.tar.gz`


## ASR Modes

All of the ASR configuration files can be found in `/SpeechRecognition/ros_test/asr`.

There are two modes of operation - *Keyword Spotting* (KWS) and *Language Model* (LM).

### KWS

Keyword spotting tries to detect specific keywords or phrases, without imposing any type of grammer rules ontop.
Utilizing keyword spotting requires a .dic file and a .kwslist file.

The dictionary file is a basic text file that contains all the keywords and their phonetic pronunciation, for instance:

```
BACK	B AE K
FORWARD	F AO R W ER D
FULL	F UH L
```

These files can be generated [here](http://www.speech.cs.cmu.edu/tools/lextool.html) . 

The .kwslist file has each keyword and a certain threshold, more or less corresponding to the length of the word or phrase, as follows:

```
BACK /1e-9/
FORWARD /1e-25/
FULL SPEED /1e-20/
```

### LM

Language model mode additionally imposes a grammer. To utilize this mode, .dic, .lm and .gram files are needed.

The dictionary file is the same as in KWS mode.

The .lm file can be generated, along with the .dic file, from a corpus of text, using [this tool](http://www.speech.cs.cmu.edu/tools/lmtool-new.html)

Finally, the .gram file specifies the grammer to be imposed. For instance, if the commands we are expecting are always an action followed by an object or person and then a location, it might look like:

```
public <rule> = <actions> [<objects>] [<names>] [<locations>];

<actions> = MOVE | STOP | GET | GIVE

<objects> = BOWL | GLASS

<names> = JOE | JOEBOB

<locations> = KITCHEN | BEDROOM

```




