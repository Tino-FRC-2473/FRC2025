# Tools & Troubleshooting on the Raspberry Pi
### *A quick guide to troubleshooting issues and navigating the Raspberry Pi*

## Systemctl
*Commands for interacting with and modifying systemd services*

Systemd services, for our FRC team, are mostly used for automatically running programs when the Raspberry Pi starts up

For the most part, you can use the following for interacting with services
```
sudo systemctl [command] [service]
```
Replace `[command]` with the command you would like to run. Replace `[service]` with the service to run the command on. 

| Command | Function                              |
| ------- | ------------------------------------- |
| start   | Starts the service                    |
| stop    | Stops the service                     |
| enable  | Run the service on  boot              |
| disable | Stop the service from running on boot |
| restart | Restarts the service                  |
| status  | Get the current status of a service   |

Note that `enable` and `disable` do not start or stop the service, they only control if the service runs on boot or not. You will need to add the `--now` flag to instantly start/stop the service.

For example, to enable and start a service called `cv-reef`, you would run
```
sudo systemctl enable --now cv-reef.service
```

Additionally, all systemd services can be listed using 
```
systemctl list-unit-files
```

If making changes to systemd services directly, you can find them in the `/etc/systemd/system/` directory. Make sure to run `systemctl daemon-reload` afterwards, and the system will prompt you to do so.

## Journalctl
*Viewing logs from systemd-journald*


To view real-time logs for a particular service, run
```
journalctl -fxu [service]
```

A quick breakdown of what each flag does
| Flag           | Function                                       |
| -------------- | ---------------------------------------------- |
| `-f`           | Follows logs in real time                      |
| `-x`           | Shows additional explanations for log messages |
| `-u [service]` | Filters logs for a specific service            |

Some other useful commands when using journalctl
| Command                     | Function                                                   |
| --------------------------- | ---------------------------------------------------------- |
| `journalctl`                | Displays entire system log (Useful if you want to grep it) |
| `journalctl -f`             | Follows all system logs in real-time                       |
| `journalctl --disk-usage`   | Check the disk usage of the logs                           |
| `journalctl --since [time]` | View logs after a specific time                            

## Vcgencmd
*is a command line utility that can get various pieces of information from the VideoCore GPU on the Raspberry Pi.*


To access info you can use
```
vcgenmd [command] 
```

A quick breakdown of what each flag does. (These flags come from the get_throttle command )
| Flag           | Function                                       |
| -------------- | ---------------------------------------------- |
| `0` |Under-voltage detected             |
| `1` |Arm frequency capped     
|`2` | Currently throttled            | `3` | Soft temperature limit active 
|`16` |Under-voltage has occured
|`17` |Arm frequency capping has occurred
|`18` |Throttling has occurred
|`19` |Soft temperature limt has occured
|




Some other useful commands when using vcgencmd (To find the more specific commands for different parts of the pi reference this link: [//www.tomshardware.com/how-to/raspberry-pi-benchmark-vcgencmd](https://phoenixnap.com/kb/dmesg-linux#:~:text=dmesg%20is%20used%20to%20check,system%20startup%20or%20connectivity%20problems.))
| Command                     | Function                                                   |
| --------------------------- | ---------------------------------------------------------- |
| `measure_temp`                | gets the temperature of the pi. (Useful to help prevent overheating of the pi) |
| `measure_clock`             |Gives options to monitor different clock speeds                       |
| ` measure_volts`   | Mesaures the voltages                 |
| `get_throttled` | reports the the throttled status of the pi. (If the pi hits around 80 degrees then it enters themral throttle. This reduces CPU speed to cool down pi)                            |
## Dmesg
*Displays messages stored in the kernel ring buffer(information of important events). Used to check for hardware errors or driver issues when troubleshooting system startup or connectivity problems*

To access dmesg, use
```
sudo dmesg 
```

A quick breakdown of what each flag does.
| Flag           | Function                                       |
| -------------- | ---------------------------------------------- |
| `-L` |Colorizes output  
|`--color = never` | turns of the off the colored output            | `--follow` | monitors the kernal ring buffer in real-time
|`-i usb` |finds buffer message about usb
|`-i tty` |finds buffer message about serial ports
|`-i eth` |finds buffer message about network
|`-i sda` |finds buffer about hard drives
|




Some other useful commands when using dmesg (Link to resource:[//www.tomshardware.com/how-to/raspberry-pi-benchmark-vcgencmd](https://phoenixnap.com/kb/dmesg-linux#:~:text=dmesg%20is%20used%20to%20check,system%20startup%20or%20connectivity%20problems.))
| Command                     | Function                                                   |
| --------------------------- | ---------------------------------------------------------- |
|
 `less`                | pipe less, allows the use of search function to loccate and highlight items |
| `grep` |                      |
| ` ` |               |
| ` ` |                            |

## To Be Added
*This documentation is a work in progress, more will be added soon*

- Top (htop, iotop, jnettop, etc)
- vcgencmd
- Basic networking (nmtui)
- dmesg (Very short section so grep will probably go here as well)
- Filesystem repairs and proper shutdowns