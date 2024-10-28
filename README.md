# Project Setup Instructions

This project requires a Python virtual environment to manage dependencies. Follow the instructions below to set up your environment.

## Requirements

- Python 3.x
- `pip` (Python package installer)

## Setting Up the Virtual Environment

1. **Clone the repository (if applicable)**

    ```sh
    git clone <repository-url>
    cd <repository-directory>
    ```

2. **Create and set up the virtual environment**

    Run the provided setup script to create a virtual environment named `MuJoCoSandBox` and install all dependencies from `requirements.txt`.

    ```sh
    ./buildEnv.sh
    ```

    **Note:** Make sure the script `buildEnv.sh` has execution permissions. If not, you can add execute permissions with:

    ```sh
    chmod +x buildEnv.sh
    ```

3. **Activate the virtual environment**

    - On Linux:

      ```sh
      source .venv/bin/activate
      ```

4. **Install dependencies**

    The setup script will automatically install all dependencies listed in the `requirements.txt` file. If you need to manually install or update dependencies, use:

    ```sh
    pip install -r requirements.txt
    ```

5. **Deactivate the virtual environment**

    Once you are done working in the virtual environment, you can deactivate it by running:

    ```sh
    deactivate
    ```