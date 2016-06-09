# Robotics: Fundamentals 2016

## Orange Team - TU Berlin

The codingstyleguide of this project is from https://www.kernel.org/doc/Documentation/CodingStyle.

## Pre-commit hook

This runs the commands from every assignment to make sure that everything compiles before committing. This measure is to avoid errors on the master branch. If catkin_make fails the commit won't take effect.

Assignment commands: `cd~/catkin_ws; catkin_make clean; catkin_make`

### Installation

To install this hook please follow the following instructions from the command line.

1. `cd` to root directory of this project

2. Given that .git folder is not tracked by git we have to make a symbolic link from our tracked file to the pre-commit hook file`ln -s ../../hooks/pre-commit .git/hooks/pre-commit`. 

   **Note:** `../../` is used as a prefix on the first argument when making a symbolic link because the path is relative to `.git/hooks/pre-commit`





## Sound_play

We chose the sound_play package to reproduce sounds and songs because it offers more flexibility on what and how it reproduces on the robot rather than plain notes as in the traditional _Store Song_ and _Play Song_ functions.
For more info go to http://wiki.ros.org/sound_play/

### Before you start

In Section ***Configuring the Sound Driver to Use Your Speaker*** of the ROS Tutorial for this sound_play  available under http://wiki.ros.org/sound_play/Tutorials/ConfiguringAndUsingSpeakers you should select your preferred input and output source. They suggest using *asoundconf*, unfortunately this tool is no longer supported. Instead you should use *Pulse Audio*. Next, the steps to take instead of the ones in the previous mentioned section.

This steps were taken from https://fleshandmachines.wordpress.com/2016/04/23/ros-and-sounds-or-how-to-make-your-droid-to-say-the-dalek-exterminate/ .

1. Run `$ pacmd list-sources` to list your input/output options
2. Set the desired input source with`$ pacmd set-default-source <INDEX OF YOUR INPUT SOURCE>`
3. Set the desired output source with`$ pacmd set-default-sink <INDEX OF YOUR OUTPUT SOURCE>`

### Installation

The installation process is not very well documented in the ROS tutorials. There are three notable tutorials for installing *sound_play*. 

1. The easiest to find is http://wiki.ros.org/sound_play/Tutorials/ConfiguringAndUsingSpeakers . It is horribly documented and outdated. Good luck!
2. You cand find another one under the page http://wenku.baidu.com/view/0b4285430b4e767f5acfce60.html, although it is in chinese. Have fun!
3. The one that worked for me was https://fleshandmachines.wordpress.com/2016/04/23/ros-and-sounds-or-how-to-make-your-droid-to-say-the-dalek-exterminate/ . 

The best tutorial is summarized as follows: 

1. `$ sudo apt-get install ros-indigo-sound-play`
2. `$ rosdep install sound_play`
3. `$ rosmake sound_play`



### Usage

Three simple steps to let the robot speak.

1. Start *roscore*
2. On second terminal run `$ rosrun sound_play soundplay_node.py`
3. On a third one run `$ rosrun sound_play say.py "Hello World"`

You should now hear **hello world**.

Enjoy!

