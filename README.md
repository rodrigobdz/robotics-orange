# Robotics: Fundamentals 2016

## Orange Team - TU Berlin

The codingstyleguide of this project is from https://www.kernel.org/doc/Documentation/CodingStyle.

## Pre-commit hook

This runs the commands from every assignment to make sure that everything compiles before committing. This measure is to avoid errors on the master branch. If catkin_make fails the commit won't take effect.

Assignment commands: `cd ~/catkin_ws; catkin_make clean; catkin_make`

### Installation

To install this hook please follow the following instructions from the command line.

1. `cd` to root directory of this project

2. Given that .git folder is not tracked by git we have to make a symbolic link from our tracked file to the pre-commit hook file`ln -s ../../hooks/pre-commit .git/hooks/pre-commit`.

   **Note:** `../../` is used as a prefix on the first argument when making a symbolic link because the path is relative to `.git/hooks/pre-commit`

