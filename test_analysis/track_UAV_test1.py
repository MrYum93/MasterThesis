#!/usr/bin/env python
# /***************************************************************************
# Copyright (c) 2018, Mathias W. Madsen <matam14@student.sdu.dk> <mwittenm@gmail.com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ****************************************************************************
'''
This is a script is to track a UAV on a video

Revision
YYYY-MM-DD
2019-03-21 MW First version
'''

import cv2
import numpy as np

class TrackUAV(object):
    def __init__(self):
        self.video_fps = 30
        self.video_one_path = '../data/test1/IMG_0090.MOV'
        self.video_one_len = 2.8
        self.video_one_max_seq = self.video_fps * self.video_one_len - 1

        self.video_two_path = '../data/test1/IMG_0093.MOV'
        self.video_two_len = 16.90
        self.video_two_max_seq = self.video_fps * self.video_two_len - 1

        self.tracker = cv2.TrackerTLD_create() # For vid 1
        # self.tracker = cv2.TrackerCSRT_create()  # For vid 2

        self.center_of_bbox = []
        self.frame_list = []

        pass

    def create_tracker_at_tip_of_uav(self, frame):
        # bbox = (287, 23, 86, 320)
        bbox = cv2.selectROI(frame, False)
        # bbox = cv2.selectROIs(frame, False)
        ok = self.tracker.init(frame, bbox)
        return ok

    def update_tracker(self, frame):
        ret, bbox = self.tracker.update(frame)

        if ret:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (0, 0, 0), 2, 1)
            x = int(bbox[0] + (bbox[2]/2))
            y = int(bbox[1] + (bbox[3]/2))
            self.center_of_bbox.append([x, y])
        else:
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    def open_video(self, path):
        cap = cv2.VideoCapture(path)
        return cap

    def display_all_centers(self, cap, frame_list):
        center_list = []
        for frame_no in frame_list:
            cap.set(1, frame_no)  # 1 is to tell opencv that we wish to display a single frame
            ret, frame = cap.read()

            # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame_draw_cnt = frame_no - frame_list[0] + 1
            center = (self.center_of_bbox[frame_draw_cnt][0], self.center_of_bbox[frame_draw_cnt][1])
            # cv2.circle(gray, center, 3, (0, 0, 255), -1)
            center_list.append(center)

            # window_name = 'frame no ' + str(frame_no)
            # cv2.imshow(window_name, gray)
            # if cv2.waitKey(0) & 0xFF == ord('q'):
            #     break
        return center_list

    def get_bbox(self, gray):
        bbox = cv2.selectROI(gray, False)  # argument 2 from the bbox is the width
        print(bbox)

    def calc_speed(self, pos_list_):
        '''
        :param pos_list_:
        :return:
        '''
        fw_len_m = 0.975  # This is from the datasheet

        fw_len_pixel_vid_one = 519
        one_px_in_m = fw_len_m / fw_len_pixel_vid_one

        # fw_len_pixel_vid_two = 239
        # one_px_in_m = fw_len_m / fw_len_pixel_vid_two

        vel_list = []
        old_pos = pos_list_[0]
        for pos in pos_list_:
            if pos == pos_list_[0]:  # cont bc we only have one frame here
                continue
            # print("pos", pos)
            delta_x = pos[0] - old_pos[0]
            delta_y = pos[1] - old_pos[1]
            delta_pos = (delta_x**2 + delta_y**2)**+0.5
            # print("delta_pos", delta_pos)
            vel_in_meters = (delta_pos * one_px_in_m) / (1 / self.video_fps)
            # print("vel_in_meters", vel_in_meters)

            vel_list.append(vel_in_meters)
            old_pos = pos

        sum = 0
        for vel in vel_list:
            sum += vel
        average = sum / len(vel_list)
        print("vel of plane list: ", vel_list)
        print("vel of plane", average)

        # cv2.imshow('plank', gray)
        # cv2.circle(gray, (bbox[0], bbox[1]), 3, (0, 0, 255), -1)
        # cv2.circle(gray, (bbox[0] + bbox[2], bbox[1]), 3, (0, 0, 255), -1)
        # cv2.line(gray, (bbox[0], bbox[1]), bbox[0] + bbox[2], [0,0,255])

    def main(self):
        path_to_video = self.video_one_path
        cap = self.open_video(path_to_video)

        frame_cnt = 19
        # frame_cnt = 422
        cap.set(1, frame_cnt)  # 1 is to tell opencv that we wish to display a single frame

        ret, frame = cap.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # self.get_bbox(gray)  # this can be used to get length of different things

        self.create_tracker_at_tip_of_uav(gray)

        while cap.isOpened():
            cap.set(1, frame_cnt)  # 1 is to tell opencv that we wish to display a single frame
            ret, frame = cap.read()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            self.update_tracker(gray)

            window_name = 'Frame no in while loop ' + str(frame_cnt)
            cv2.namedWindow(window_name, cv2.WINDOW_FULLSCREEN)
            cv2.imshow(window_name, gray)

            key = cv2.waitKey(0)

            if key == ord('q'):
                print('you decided to quit at frame', frame_cnt)
                break
            elif key == ord('a'):
                print('Going back one frame')
                self.center_of_bbox.pop()
                frame_cnt -= 1
            elif key == ord('d'):
                print('Going forward one frame')
                frame_cnt += 1

            # cv2.destroyAllWindows()

            self.frame_list.append(frame_cnt)

        center_list = self.display_all_centers(cap, self.frame_list)
        self.calc_speed(center_list)

        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    test = TrackUAV()
    test.main()
