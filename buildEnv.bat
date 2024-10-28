@echo off
REM This BAT file creates, activates a Python virtual environment, installs dependencies, and checks for executable scripts

REM Set the environment name (you can change 'venv' to any name you prefer)
set ENV_NAME=venv

REM Print starting message
echo ==============================================
echo Starting the virtual environment setup process
echo ==============================================

REM Check if Python is installed
python --version >nul 2>&1
IF ERRORLEVEL 1 (
    echo [ERROR] Python is not installed or not in your PATH.
    pause
    exit /b
)

REM Print Python check success
echo [INFO] Python is installed and detected.

REM Create the virtual environment
echo [INFO] Creating the virtual environment: %ENV_NAME%...
python -m venv %ENV_NAME%

REM Check if the environment was created successfully
IF EXIST "%ENV_NAME%\Scripts\activate.bat" (
    echo [SUCCESS] Virtual environment created successfully.

    echo [INFO] Activating the virtual environment...
    call %ENV_NAME%\Scripts\activate.bat

    REM Install dependencies from requirements.txt if it exists
    IF EXIST requirements.txt (
        echo [INFO] requirements.txt found. Installing dependencies...
        pip install -r requirements.txt
        IF %ERRORLEVEL% EQU 0 (
            echo [SUCCESS] Dependencies installed successfully.
        ) ELSE (
            echo [ERROR] Failed to install dependencies. Please check requirements.txt.
        )
    ) ELSE (
        echo [WARNING] requirements.txt not found. Skipping dependency installation.
    )

    REM Make a script executable (Windows equivalent)
    IF EXIST runScript.bat (
        echo [INFO] runScript.bat found. No need for chmod on Windows.
        echo [INFO] The script is ready to be executed as needed.
    ) ELSE (
        echo [WARNING] runScript.bat not found. Skipping the executable step.
    )

) ELSE (
    echo [ERROR] Failed to create the virtual environment. Please check for errors.
)

REM Print completion message
echo ==============================================
echo Process completed. Review any messages above.
echo ==============================================

REM Pause to keep the command prompt open
pause
