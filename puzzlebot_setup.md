# Puzzlebot setup
To ensure smooth operation of the puzzlebot, we need to install several packages and libraries. This includes setting up ROS 2 Humble along with the corresponding workspace for ROS 2.

## Explanation
Initially, we aim to execute the setup.sh script on our computers/laptops. This allows us to test nodes, launch files, and make progress independent of the Jetson. The shell script automates the installation of ROS 2, creation of the workspace, installation of required packages, and necessary libraries. This ensures proper functionality of the Lidar, enables camera detection of Arucos, and provides Gazebo for simulation.

```
set -e  # Exit immediately if a command exits with a non-zero status
```
First we have the `set -e` command, this command sets the shell option `-e`, which stands for "errexit". When `-e` is enabled, the shell will exit immediately if any command exits with a non-zero status, indicating an error. This helps in ensuring that the script stops execution upon encountering errors, preventing it from continuing in an erroneous state.

```
# Logging
LOG_FILE="install_ros2.log"
exec > >(tee -a "$LOG_FILE") 2>&1
```
In the script, the `exec > >(tee -a "$LOG_FILE") 2>&1` line redirects all output (both standard output and standard error) of subsequent commands to a specified log file `($LOG_FILE)`. Let's break down this line:
- `exec`: This command allows redirecting the shell's file descriptors.
- `> >(tee -a "$LOG_FILE")`: This part redirects the output to both the console and the log file. It uses process substitution `(> >(...))` to create a subshell where the tee command is run. tee reads from its standard input and writes to both the file specified `("$LOG_FILE")` and the standard output.
- `2>&1`: This part redirects standard error (file descriptor 2) to the same place as standard output (file descriptor 1), which ensures that both standard output and standard error are captured and logged.

By logging the output, you can monitor the progress of the script and review any errors or warnings that occur during execution. It's a helpful practice for debugging and troubleshooting purposes, especially in automated or unattended script executions.

For the next parts it's just the normal installation of the ROS2 Humble, the ros2_workspace and the packages, but we use the `-y` flag, in case you didn't know, the `-y` flag is used with the `apt install` command in Ubuntu and Debian-based Linux distributions. When you run `apt install` to install packages, it typically prompts you to confirm whether you want to proceed with the installation.

Adding the `-y` flag instructs `apt` to automatically answer "yes" to all prompts, essentially telling it to assume "yes" as the answer to any confirmation questions. This is useful in scripts where user interaction is not desired or possible, as it prevents the installation process from halting to wait for manual confirmation.

For example, if you run `sudo apt install -y package_name`, it will install the specified package without asking for confirmation. This is particularly helpful in automated scripts like the installation script we've been discussing, where you want the installation to proceed without requiring user intervention.

And that's it, there are more complex commands but for this project we are not going to dive into those. Have a fun time coding!