# Mecha-2025
Mecanum for the 2025 season
## Guide

### Install dependencies

1. Install [Python 3.12.8](https://www.python.org/downloads/)
2. Install [Chocolatey](https://chocolatey.org/install) as admin
3. Install Make as admin

    ```console
    choco install make
    ```

4. Install [VS Code](https://code.visualstudio.com/download)
5. Install [Git](https://git-scm.com/downloads)
6. Clone repository

    ```console
    git clone https://github.com/km-otternauts-6758/Mecha-2025.git
    ```

7. Open source code directory
8. Run `make computer_install` when connected to the internet
9. Run `make roborio_install` when connected to the roboRIO

### Commands
- `make deploy`: Deploys the code to the roboRIO
- `make delete`: Deletes the code on the roboRIO
- `make sync`: Syncs all the files for the robot libraries
