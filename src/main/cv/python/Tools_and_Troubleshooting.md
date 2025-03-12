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
| `journalctl --since [time]` | View logs after a specific time                            |


## To Be Added
*This documentation is a work in progress, more will be added soon*

- Top (htop, iotop, jnettop, etc)
- vcgencmd
- Basic networking (nmtui)
- dmesg (Very short section so grep will probably go here as well)
- Filesystem repairs and proper shutdowns