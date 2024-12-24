# with-robot-3rd

With Robot Lab Season 3

## Install CoppeliaSim

From [CoppeliaSim homepage](https://www.coppeliarobotics.com), download CoppeliaSim Edu installer. (version 4.7.0 is recommended.)

Run the installer and complete installation.

## Install Miniconda (optional)

From [Miniconda homepage](https://docs.anaconda.com/miniconda/install/), download Miniconda installer.

Run the installer and complete installation.

## Create conda environment (optional)

Type below commands in terminal.

The conda environment name "PICABOT" can be replaced.

    conda create -n PICABOT python=3.10 -y

    conda activate PICABOT

## Install required python packages

Type below commands in terminal.

For Ubuntu, 

    pip install -r requirements_PICABOT_Ubuntu.txt

For Windows, 

    pip install -r requirements_PICABOT_Windows.txt

## Install git bash ( + git client tool)

From [Git homepage](https://git-scm.com/downloads), download git bash installer.

Run the installer and complete installation.

## Clone PICABOT repository

Type below commands in terminal or git bash.

    git clone git@github.com:with-robot/with-robot-3rd.git

If there is no github authentication registered, type below instead.

    git clone https://github.com/with-robot/with-robot-3rd.git

## Run PICABOT simulation

Run CoppeliaSim and open the scene

![CoppeliaSim PiCABOT scene0](/resources/images/CoppeliaSim_PICABOT_scene0.png)

Run simulation by typing commands in terminal like below.

    python main.py

![command to run PiCABOT simulation](/resources/images/terminal_PICABOT_command.png)

## Modify the project (optional)
 