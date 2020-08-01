
import argparse
import rospy
import rosbag

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

import cv2
import cv_bridge
from spath import Path

parser = argparse.ArgumentParser(description='viz')
parser.add_argument('bag', type=str, help="show bag")

args = parser.parse_args()

bridge = cv_bridge.CvBridge()

bagfname = Path(args.bag).expand()
parent_dir = bagfname.parent
annot_dir = (parent_dir/bagfname.namebase).makedirs_p()

bag = rosbag.Bag(bagfname, 'r')

img = None
throttle = None
steer = None

isgood = False

W = 640
H = 480

wait_ms = 100

ix = -1
for topic, msg, ts in bag.read_messages():
    # bag.write(topic, msg, ts)
    ix += 1

    if topic == '/camera/color/image_raw/compressed':
        img = bridge.compressed_imgmsg_to_cv2(msg)
    elif topic == '/camera/color/image_raw':
        img = bridge.imgmsg_to_cv2(msg)
    elif topic == '/cmd_vel':
        throttle = msg.linear.x
        steer = msg.angular.z

    if img is not None:
        if steer is None:
            continue

        drawimg = img.copy()

        rectx0 = 20
        rectx1 = W-20
        recty0 = H-40
        recty1 = H-20

        drawimg = cv2.rectangle(drawimg, (rectx0, recty0), (rectx1, recty1), (0, 255, 0), 3)

        steer01 = (steer + 1)/2.
        lx = int(rectx0*steer01 + rectx1*(1-steer01))
        drawimg = cv2.line(drawimg, (lx, recty0), (lx, recty1), (0, 0, 255), 3)

        if isgood:
            drawimg = cv2.circle(drawimg, (W-40, 40), 20, (0, 0, 255), -1)
        else:
            drawimg = cv2.circle(drawimg, (W-40, 40), 20, (0, 0, 255), 1)

        cv2.imshow('img', drawimg)
        k = cv2.waitKey(wait_ms)
        if k == 27:
            cv2.destroyAllWindows()
            break
        elif k == 32:
            isgood = not isgood
        elif k == 43:
            print('+')
            wait_ms = max(wait_ms/2, 1)
        elif k == 45:
            print('-')
            wait_ms = wait_ms*2
        elif k == 61:
            print('=')
            wait_ms = 100

        if isgood:
            annot_img_fname = annot_dir/('%s@%06d.jpg' % (bagfname.namebase, ix))
            cv2.imwrite(str(annot_img_fname), img)
            annot_controls_fname = annot_dir/('%s@%06d.json' % (bagfname.namebase, ix))
            annot_controls_fname.write_json({'throttle': throttle, 'steer': steer})

        img = None


bag.close()
