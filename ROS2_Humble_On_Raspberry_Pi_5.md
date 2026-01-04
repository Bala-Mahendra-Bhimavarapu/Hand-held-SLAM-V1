# ROS2 Humble Installation Script for Raspberry Pi 5 on Bookworm OS

This repository contains a shell script (`ros2_humble_install.sh`) to automate the installation of ROS2 Humble on a Raspberry Pi 5 running Debian Bookworm OS. It installs the necessary dependencies, downloads the ROS2 source code, and builds it.

## Prerequisites

Before running the script, ensure the following:

- You have a Raspberry Pi 5 running the **Debian Bookworm OS**.
- You have **sudo** privileges to install packages and update system configurations.

## How to Run

1. **Clone the Repository**

   Clone this repository to your Raspberry Pi 5:

   ```bash
   git clone https://github.com/ozandmrz/ros2_raspberry_pi_5
   cd ros2_raspberry_pi_5
   ```

2. **Make the Script Executable**

   Ensure the script has executable permissions:

   ```bash
   chmod +x ros2_humble_install.sh
   ```

3. **Run the Script**

   Execute the script to begin the installation process:

   ```bash
   ./ros2_humble_install.sh
   ```

   The script will:
   - Install necessary dependencies.
   - Set up a new ROS2 workspace.
   - Download the ROS2 Humble source code.
   - Compile ROS2.
   - Configure the environment variables to automatically source the setup file.

4. **Verify the Installation**

   After the installation, you can check if ROS2 is properly installed by running:

   ```bash
   source ~/ros2_humble/install/local_setup.bash
   ros2 --version
   ```

   This should display the installed ROS2 version.

## Notes

- The script will automatically update your `~/.bashrc` to source the ROS2 setup file each time you open a new terminal.
- If the script fails due to missing dependencies or any system configuration, ensure that your system is up-to-date and try again.
