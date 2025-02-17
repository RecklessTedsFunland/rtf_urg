#!/usr/bin/env python3
# https://github.com/foxglove/foxglove-sdk/tree/main/schemas/proto/foxglove
#
# pip install opencv-contrib-python websockets
# pip install websockets numpy opencv-python protobuf
# pip install foxglove-websocket[examples]
#
import asyncio
import sys
import time
from base64 import b64encode
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServerListener
from foxglove_websocket.server import FoxgloveServer
from foxglove_schemas_protobuf.LaserScan_pb2 import LaserScan
from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
from google.protobuf.descriptor_pb2 import FileDescriptorSet
from google.protobuf.descriptor import FileDescriptor
from pyquaternion import Quaternion
import numpy as np
import cv2
from lidar_urg import URG04LX, Scan
from threading import Thread


bgr2gray = lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
deg2rad = np.pi / 180

global_run = True
pts = Scan([],0)
lidar = URG04LX()


def lidar_worker(port):
  global global_run
  global pts
  global lidar

  ok = lidar.init(port, baudrate=19200)
  if not ok:
    print("*** Couldn't init lidar ***")
    sys.exit(1)

  print(lidar.pp_params)
  lidar.printInfo()

  while global_run:
    pts = lidar.capture()
    time.sleep(0.1)

  lidar.cleanup()


def build_file_descriptor_set(message_class):
  """
  Build a FileDescriptorSet representing the message class and its dependencies.
  """
  file_descriptor_set = FileDescriptorSet()
  seen_dependencies: Set[str] = set()

  def append_file_descriptor(file_descriptor):
    for dep in file_descriptor.dependencies:
      if dep.name not in seen_dependencies:
        seen_dependencies.add(dep.name)
        append_file_descriptor(dep)
    file_descriptor.CopyToProto(file_descriptor_set.file.add())

  append_file_descriptor(message_class.DESCRIPTOR.file)
  return file_descriptor_set


async def main():
  class Listener(FoxgloveServerListener):
    async def on_subscribe(self, server, channel_id):
      print("First client subscribed to", channel_id)

    async def on_unsubscribe(self, server, channel_id):
      print("Last client unsubscribed from", channel_id)

  async with FoxgloveServer("0.0.0.0", 8765, "example server") as server:
    server.set_listener(Listener())
    chan_id = await server.add_channel(
      {
        "topic": "lidar_msg",
        "encoding": "protobuf",
        "schemaName": LaserScan.DESCRIPTOR.full_name,
        "schema": b64encode(
            build_file_descriptor_set(LaserScan).SerializeToString()
        ).decode("ascii"),
        "schemaEncoding": "protobuf",
      }
    )

    tf_id = await server.add_channel(
      {
        "topic": "tf_msg",
        "encoding": "protobuf",
        "schemaName": FrameTransform.DESCRIPTOR.full_name,
        "schema": b64encode(
            build_file_descriptor_set(FrameTransform).SerializeToString()
        ).decode("ascii"),
        "schemaEncoding": "protobuf",
      }
    )

    i = 0
    while True:
      i += 1
      await asyncio.sleep(0.05)
      now = time.time_ns()

      q = Quaternion(axis=[0, 0, 1], angle=0.0)

      T = FrameTransform()
      T.timestamp.FromNanoseconds(now)
      T.parent_frame_id = "root"
      T.child_frame_id = "lidar"
      T.translation.x = 0
      T.translation.y = 0
      T.translation.z = 0
      T.rotation.x = q.x
      T.rotation.y = q.y
      T.rotation.z = q.z
      T.rotation.w = q.w
      await server.send_message(tf_id, now, T.SerializeToString())

      # LIDAR ---------------------------------------------
      scan = LaserScan()
      scan.frame_id = "lidar"
      scan.timestamp.FromNanoseconds(now)
      scan.pose.orientation.x = q.x
      scan.pose.orientation.y = q.y
      scan.pose.orientation.z = q.z
      scan.pose.orientation.w = q.w
      scan.start_angle = lidar.angles.min*deg2rad
      scan.end_angle = lidar.angles.max*deg2rad
      for pt in pts.scan:
        scan.ranges.append(pt)
        scan.intensities.append(pt)
      await server.send_message(chan_id, now, scan.SerializeToString())

if __name__ == "__main__":
  if len(sys.argv) != 2:
    print("ERROR: No serial port given")
    print("Usage: {sys.argv[0]} <port>")
    sys.exit(1)

  port = sys.argv[1]

  lidar_thread = Thread(target=lidar_worker, args=(port,))
  lidar_thread.start()

  run_cancellable(main())

  global_run = False

  lidar_thread.join()
  print("this is the end")