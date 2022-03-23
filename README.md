# rpi_led_consensus

## Instalation
1. Navigate to your catkin ws, e.g. `cd ~/catkin_ws/src`.
2. Clone this repository: `git clone git@github.com:mkrizmancic/rpi_led_consensus.git`
3. Install [RPi LED library](https://github.com/mkrizmancic/rpi_ws281x_pylib) using provided instructions. Clone that repository to `catkin_ws/src` (not "wherever" as it says in the instructions).
4. Navigate one level up, i.e. `cd ~/catkin_ws` and run `catkin_make`.
5. Do this on every computer involved.

## Usage
For this to work, ROS has to run remotely. See the instructions at the end of the file.

### Option 1 (recommended)
1. Install tmuxinator: `sudo apt install tmuxinator`
1. Set up passwordless SSH for Raspberries by following [this tutorial](https://www.linuxtechi.com/passwordless-ssh-login-keys-linux/)
1. Set up parameters in `rpi_led_consensus/params/config.yaml`
1. From the root package directory run `terminator start -p config/rpi_led.yml`. This will open Tmux with two windows, one for local stuff, and one for 4 Raspberries. It will automatically start ROS, load parameters, SSH to Raspberies and run necessary scripts on them. You can use [this cheatsheet](https://tmuxcheatsheet.com/) to learn how to navigate in Tmux.
1. When you want to exit, select the window with Raspberries and pres Ctrl-c. Then, press the Tmux prefix (Ctrl-b) and type ":kill-session"

### Option 2
1. Set up parameters in `rpi_led_consensus/params/config.yaml`
1. On your PC, start a ROS core and load default parameters by running `roslaunch rpi_led_consensus main.launch`
1. On each Raspberry, run `bash <path_to_rpi_led_consensus>/launch/starter.sh`
1. You can use service calls to `/rpiX/set_random` and `/rpiX/set_value` to set random or specified color values.
1. It is recommended that you use some smarter terminal like [Terminator](https://terminator-gtk3.readthedocs.io/en/latest/) for all of this.



## Working with ROS remotely
When working with ROS remotely, one computer acts as master through which all others communicate. Usually we set our personal computer to be the master. In order for ROS to work, each computer must know its own address as well as the address of the master.

### Edit .bashrc file on your Raspberry Pis
Open the .bashrc file on your Raspberry Pi using nano: `nano ~/.bashrc` and find the following lines:
```bash
export NAMESPACE=$HOSTNAME
export ROS_MASTER_URI=http://192.168.0.199:11311
export ROS_IP=192.168.0.20
```
Set the IP address in ROS_MASTER_URI to IP address of your PC/laptop. Alternatively, instead of IP address, it might be possible to put the hostname in form of `<your_hostname>.local`.  

Set the ROS_IP address to the IP address of the Raspberry Pi. Alternatively, remove the line with ROS_IP and instead put `export ROS_HOSTNAME=$HOSTNAME.local`

### Edit .bashrc file on your PC/laptop
Open the .bashrc file on your PC using your favourite editor, e.g. `gedit ~/.bashrc` and find the same lines as before.  
In both ROS_MASTER_URI and ROS_IP / ROS_HOSTNAME set the IP address or hostname of your PC.

### Connecting and using Raspberry Pis
Unless you have x amount of spare monitors and keyboards, you will have to connect to Raspberries remotely from your PC using SSH. Before anything, make sure that your PC and Raspberries are connected to the same network. By default, they are looking for the network "RPi_WIFI" with password "demo_rpi" which you can create, for example, with a hotspot on your phone. Once connected, you can add your home network as well.

**If you don't have ssh installed:**
```bash
sudo apt install openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh
```
**To connect:** `ssh pi@rpi0.local` and enter password on prompt.
