#!/bin/bash

# Create the virtual environment named G1-SRBD-MPC
python3 -m venv .venv

# Activate the virtual environment
source .venv/bin/activate

pip install --upgrade pip

# Install the dependencies from requirements.txt
pip install -r requirements.txt

chmod +x ./runScript.sh

echo "Virtual environment 'venv' is set up and dependencies are installed."