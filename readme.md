# NYU Finger robot

## What it is

Low level interface for the NYU Finger robot.

## Running the python demo without sudo

To run the python jupyter notebook demo without sudo, you have to adjust the permissions for the python executable:

```
$ sudo setcap cap_net_admin,cap_net_raw+ep /usr/bin/python2.7
```

Also, make sure the user is part of the `realtime` group.

```
# To check if the user is part of the realtime group
$ groups

# To add the current user to the realtime group
# You will need to logout and login the current user again
# after running this command.
$ sudo usermod -a -G realtime $USER
```

## Authors

- Julian Viereck

## Copyrights

Copyright(c) 2020 Max Planck Gesellschaft, New York University

## License

BSD 3-Clause License


