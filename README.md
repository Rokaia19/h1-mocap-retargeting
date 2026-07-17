# Locomotion and Whole-Body Control

## Basics for bringing up the container

We assume the underlying system to be an Ubuntu.
Other Linux distributions should work with minor modifications (i.e. when choosing installation files/commands in 1 and 2).
Windows and OSX are currently untested/not supported, if you want to use those, you will have to see how to make this setup work on your own.

1. You need Visual Studio Code (that is not the community edition, see 2): https://code.visualstudio.com/
2. Install Docker Engine (not Docker Desktop): https://docs.docker.com/engine/install/ubuntu/. Reboot afterwards.
3. Open the provided folder in vscode. It should prompt you to install the "Dev Container" extension, if not, do so manually.
4. You should get prompted to "Reopen in Dev Container in the bottom right corner of vscode. Otherwise, press Ctrl+Shift+P and type "Reopen in Container" into the prompt.
5. You will get a terminal at the bottom that should provide a shell inside your devcontainer.

## Basics inside the container

After starting the devcontainer, you might need to source the generated bashrc via

    source install/setup.bash

This provides you with tab-completion for ROS commands, which is quite convenient.
The package we use for H1 simulation is https://github.com/K-d4wg/ros2_heinz, the code of which we already included in the container.
If you want to star it on GitHub, you are nevertheless invited to do so!

If you want to launch the setup for the first exercise, for example, that command would be

    ros2 launch sheet_1 exercise_1.launch.py

And with that, we wish you good luck with your endeavours in ROS2 and great success diving into the depth of practical robotics.
