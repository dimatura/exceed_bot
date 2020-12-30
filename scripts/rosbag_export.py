import argparse
import json
import rosbag
import cv2
import numpy as np
from tqdm import tqdm
import numpy as np

from path import Path


COLOR_TOPIC = '/camera/color/image_raw'
DEPTH_TOPIC = '/camera/aligned_depth_to_color/image_raw'


def main_default(args):
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


def main_reorder(args):
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

    color_msgs = []
    depth_msgs = []

    color_cnt = bag.get_message_count(COLOR_TOPIC)
    depth_cnt = bag.get_message_count(DEPTH_TOPIC)

    pbar = tqdm(total=(color_cnt + depth_cnt))
    for topic, msg, ts in bag.read_messages(topics=[COLOR_TOPIC, DEPTH_TOPIC]):
        if topic == COLOR_TOPIC:
            color_msgs.append(msg)
        elif topic == DEPTH_TOPIC:
            depth_msgs.append(msg)
        else:
            raise ValueError('huh?')
        pbar.update()
    pbar.close()
    print('sorting')
    color_msgs.sort(key=lambda k: k.header.stamp.to_sec())
    depth_msgs.sort(key=lambda k: k.header.stamp.to_sec())

    color_ts = np.asarray([m.header.stamp.to_sec() for m in color_msgs])
    depth_ts = np.asarray([m.header.stamp.to_sec() for m in depth_msgs])

    print('matching')
    pbar = tqdm(total=color_cnt)
    color_depth_pairs = []
    for i, cts in enumerate(color_ts):
        diff = np.abs(cts - depth_ts)
        nz = np.sum(diff == 0)
        # print('%d, eq: %d' % (i, nz))
        if nz == 0:
            continue
        nzix = np.nonzero(diff == 0)[0]
        color_depth_pairs.append((i, nzix[0]))
        pbar.update()
    pbar.close()

    print('writing')
    ctr = 0
    pbar = tqdm(total=len(color_depth_pairs))
    with open(out_dir/'rgb.txt', 'w') as cf, open(out_dir/'depth.txt', 'w') as df:
        for cix, dix in color_depth_pairs:
            cmsg = color_msgs[cix]
            dmsg = depth_msgs[dix]

            img = np.frombuffer(cmsg.data).view('u1').reshape((cmsg.height, cmsg.width, 3))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img_basename = '%06d.jpg' % ctr
            cv2.imwrite(str(color_dir/img_basename), img)
            cf.write('%d.%d %s\n' % (cmsg.header.stamp.secs, cmsg.header.stamp.nsecs, ('color/%s' % img_basename)))

            depth = np.frombuffer(dmsg.data).view('u2').reshape((dmsg.height, dmsg.width))
            depth_basename = '%06d.png' % ctr
            cv2.imwrite(str(depth_dir/depth_basename), depth)
            df.write('%d.%d %s\n' % (dmsg.header.stamp.secs, dmsg.header.stamp.nsecs, ('depth/%s' % depth_basename)))

            ctr += 1
            pbar.update()
    pbar.close()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Export rosbag to images')
    parser.add_argument('rosbag', type=str)
    parser.add_argument('--reorder', action='store_true')
    args = parser.parse_args()
    print(args)
    if args.reorder:
        main_reorder(args)
    else:
        # main_default(args)
        pass
