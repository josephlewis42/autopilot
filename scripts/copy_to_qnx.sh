#!/bin/bash
ORIG_PWD=`pwd`

# make the current autopilot source
cd ../autopilot/src/
/opt/qnx650/host/linux/x86/usr/bin/make 
cd "$ORIG_PWD"

# copy it to the remote host
cp ../autopilot/src/x86/o-g/autopilot_g "../builds/`date --rfc-3339=seconds`_autopilot_g"
scp ../autopilot/src/x86/o-g/autopilot_g ../autopilot/src/x86/o/autopilot root@10.42.0.2:/autopilot/

# start qgroundcontrol if it isn't already
if ps aux | grep "[q]groundcontrol" > /dev/null
then
    echo "QGroundControl already Running"
else
   cd /home/joseph/Desktop/QGroundControl
   ./qgroundcontrol &
fi
