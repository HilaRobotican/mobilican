#!/usr/bin/env python

import sys
import rospy
from image_snapshot.srv import *

def image_snapshot_client():
    # a client that sends a request for image snapshot.

    # This method blocks until the service named image_snapshot is available.
    rospy.wait_for_service('image_snapshot')

    try:
        image_snapshot = rospy.ServiceProxy('image_snapshot', ImageSnapshot)
        resp1 = image_snapshot()
        return resp1.sum
    except (rospy.ServiceException, e):
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, image_snapshot_client(x, y)))