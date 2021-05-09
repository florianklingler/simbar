# SimbaR
This is Simulation-based Radio (SimbaR).
Please refer to "https://www.fklingler.net/bib/franke2021hil/franke2021hil.pdf" for a
detailed description.

It is tested using Ubuntu 20.04 LTS and PC Engines APU2 single board computers equipped
with MikroTik R11e-5HnD wlan cards and PC Engines Alix single board computers.

# Prerequisite packages

In the file 'prerequisite_packages.txt' there is a list of required packages.
You can install them using this command:

"xargs -a prerequisite_packages.txt sudo apt-get install".

# List your devices in the configuration

Before building SimbaR you need to list your devices in the following file:

"config/openWRT/files/etc/rc.local"

You need 4 device. Add them with their MAC address in the same manner as the already
listed devices. Have a dedicated look at both case-statements.

# Build SimbaR

To build SimbaR please run "build.sh" which wil take some time:

"./build.sh"

It will automatically download and compile all other dependencies which are listed below

software versions:
OMNeT++: 5.5.1
Veins: 5.0
SUMO: 1.2.0
openWRT: 19.07.2
asn1c: https://github.com/zhanglei002/asn1c.git (revision 34e88d2)

# Flashing the openWRT image

After successfully building everything go to the pxe boot server in the openWRT folder:

"cd lib/openWRT/pxe_boot_server"

In this folder there is a script called "start_dnsmasq.sh". Please have a look at it and
set the variable "interface" to the correct name of your ethernet interface. Additionally,
assure that appamore is disabled. Afterwards, you can start the pxe boot server with the
following command:

"./start_dnsmasq.sh"

Turn on your hardware and let it boot.

# Running the example

First, you of to source the file "export.sh" with the following command in the SimbaR
root directory:

"source export.sh"

For running the example navigate into the folder "experiments/example" starting from the
root folder. Before executing the script make sure that the correct ethernet interface is
used in line 19 of "run.sh" and the used IP addresses suits your hardware. Afterwards,
execute the run script which automatically configurates the hardware and starts an
exemplary simulation run with this command:

"./run.sh"

The simulation results are stored in "src/examples/StaticScenario/results". The results
of the channel load measurement are stored in the home folder of the root user of the
measurement device.
