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

    Run the provided setup script to create a virtual environment named `G1-SRBD-MPC` and install all dependencies from `requirements.txt`.

    ```sh
    ./setup_env.sh
    ```

    **Note:** Make sure the script `setup_env.sh` has execution permissions. If not, you can add execute permissions with:

    ```sh
    chmod +x setup_env.sh
    ```

3. **Activate the virtual environment**

    - On Windows:

      ```sh
      G1-SRBD-MPC\Scripts\activate
      ```

    - On macOS and Linux:

      ```sh
      source G1-SRBD-MPC/bin/activate
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

## Additional Information

- **Updating dependencies:** If you need to add new packages, install them using `pip` and update the `requirements.txt` file:

  ```sh
  pip install <package-name>
  pip freeze > requirements.txt
