import argparse
import json
import rosbag
import cv2
import numpy as np

from path import Path


parser = argparse.ArgumentParser(description='Export rosbag to images')
parser.add_argument('rosbag', type=str)
args = parser.parse_args()

out_dir = Path(args.rosbag).stripext()
print(out_dir)
out_dir.makedirs_p()
color_dir = (out_dir/'color').makedirs_p()
depth_dir = (out_dir/'depth').makedirs_p()



bag = rosbag.Bag(args.rosbag)
for topic, msg, ts in bag.read_messages(topics=['/camera/color/camera_info']):
    print(msg)

    # for open3d format, it's transposed
    K2 = [msg.K[0], 0, 0, 0, msg.K[4], 0, msg.K[2], msg.K[5], 1]
    intrinsic = {'width': msg.width, 'height': msg.height, 'intrinsic_matrix': K2}
    with open(out_dir/'camera_intrinsic.json', 'w') as f:
        f.write(json.dumps(intrinsic, indent=2))

    with open(out_dir/'ldvo_intrinsic.txt', 'w') as f:
        f.write('%d %d\n' % (msg.width, msg.height))
        f.write('%f %f %f\n' % (msg.K[0], msg.K[1], msg.K[2]))
        f.write('%f %f %f\n' % (msg.K[3], msg.K[4], msg.K[5]))
        f.write('%f %f %f\n' % (msg.K[6], msg.K[7], msg.K[8]))
        f.write('0.0 0.0 0.0 0.0 0.0\n')

    break


with open(out_dir/'rgb.txt', 'w') as f:
    ctr = 0
    for topic, msg, ts in bag.read_messages(topics=['/camera/color/image_raw']):
        img = np.frombuffer(msg.data).view('u1').reshape((msg.height, msg.width, 3))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_basename = '%06d.jpg' % ctr
        cv2.imwrite(str(color_dir/img_basename), img)
        f.write('%d.%d %s\n' % (msg.header.stamp.secs, msg.header.stamp.nsecs, ('color/%s' % img_basename)))
        ctr += 1
        # break


with open(out_dir/'depth.txt', 'w') as f:
    ctr = 0
    for topic, msg, ts in bag.read_messages(topics=['/camera/aligned_depth_to_color/image_raw']):
        depth = np.frombuffer(msg.data).view('u2').reshape((msg.height, msg.width))
        depth_basename = '%06d.png' % ctr
        cv2.imwrite(str(depth_dir/depth_basename), depth)
        f.write('%d.%d %s\n' % (msg.header.stamp.secs, msg.header.stamp.nsecs, ('depth/%s' % depth_basename)))
        ctr += 1
        # break


bag.close()
