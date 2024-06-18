#!/bin/bash

git init

# Create the virtual environment named MuJoCoSandBox
python3 -m venv MuJoCoSandBox

# Activate the virtual environment
source MuJoCoSandBox/bin/activate

# Install the dependencies from requirements.txt
pip install -r requirements.txt

# Configure git user name and email
git config --global user.name "Dhruv Thanki"
git config --global user.email "thankid@udel.edu"

# Create a folder for submodules
mkdir -p submodules

# Add the repository as a submodule inside the submodules folder
git submodule add https://github.com/google-deepmind/mujoco_menagerie.git submodules/mujoco_menagerie

# Initialize and update the submodule
git submodule update --init --recursive

echo "Virtual environment 'MuJoCoSandBox' is set up, dependencies are installed, and submodule is added in the 'submodules' folder."
