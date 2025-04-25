#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import os
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
import argparse
import matplotlib.cm as cm
from scipy import interpolate
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

# Global parameters
pose_timestamp = []
pose_x_data = []
pose_y_data = []
sat_timestamp = []
sat_num_data = []

interested_topics = [
            '/localization/pose_estimator/pose',
            '/sensing/gnss/septentrio/pvtgeodetic']

class BagFileParser:
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name: msg_type for id_, name, msg_type in topics_data if name in interested_topics}
        self.topic_id = {name: id_ for id_, name, msg_type in topics_data if name in interested_topics}
        self.topic_msg_message = {name: get_message(msg_type) for name, msg_type in self.topic_type.items()}

    def __del__(self):
        self.conn.close()

    def get_messages(self, topic):
        if topic not in self.topic_id:
            print(f"Warning: Topic {topic} not found in the bag file.")
            return []
        topic_id = self.topic_id[topic]
        rows = self.cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ?", (topic_id,)
        ).fetchall()
        return rows

def process_bag_file(bag_file):
    bag = BagFileParser(bag_file)

    # Self-pose data
    rows = bag.get_messages(interested_topics[0])
    for timestamp, data in rows:
        topic_msg = deserialize_message(data, bag.topic_msg_message[interested_topics[0]])
        pose_timestamp.append(timestamp)
        pose_x_data.append(topic_msg.pose.position.x)
        pose_y_data.append(topic_msg.pose.position.y)

    # GNSS info data
    rows = bag.get_messages(interested_topics[1])
    for timestamp, data in rows:
        topic_msg = deserialize_message(data, bag.topic_msg_message[interested_topics[1]])
        sat_timestamp.append(timestamp)
        sat_num_data.append(int(topic_msg.nr_sv))

def plot_data():
    if not sat_num_data:
        print("Warning: No satellite data found.")
        vmin, vmax = 0, 1
    else:
        vmin, vmax = min(sat_num_data), max(sat_num_data)

    # Interpolate, make the data length same
    if len(sat_timestamp) > 1 and len(sat_num_data) > 1:
        num_s_interp = np.interp(pose_timestamp, sat_timestamp, sat_num_data)
        num_s_interp = np.round(num_s_interp).astype(int)
    else:
        num_s_interp = np.full(len(pose_x_data), int(sat_num_data[0]) if sat_num_data else 0)

    # Map
    fig, ax = plt.subplots()
    scatter = ax.scatter(pose_x_data, pose_y_data, c=num_s_interp, cmap=cm.jet, vmin=vmin, vmax=vmax)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_title('The number of satellites by driving location')
    colorbar = plt.colorbar(scatter)
    colorbar.set_label('Satellites Number')

    # Show annotation info
    annot = ax.annotate("", xy=(0, 0), xytext=(15, 15), textcoords="offset points",
                        bbox=dict(boxstyle="round,pad=0.3", fc="yellow", alpha=0.8),
                        arrowprops=dict(arrowstyle="->"))
    annot.set_visible(False)

    # Calculate the position for all points
    pos_xy_array = np.column_stack((pose_x_data, pose_y_data))

    def update_annot(ind):
        """Update annotation info"""
        index = ind["ind"][0]   # Get index info
        x, y = pose_x_data[index], pose_y_data[index]
        sat_num = num_s_interp[index]
        annot.xy = (x, y)
        annot.set_text(f"Sat: {sat_num:d}\n(x: {x:.2f}, y: {y:.2f})")
        annot.set_visible(True)

    def on_mouse_move(event):
        """mouse event, update the number of satellite"""
        if event.inaxes == ax:
            xy = np.array([event.xdata, event.ydata])
            distances = np.linalg.norm(pos_xy_array - xy, axis=1)
            nearest_index = np.argmin(distances)
            if distances[nearest_index] < 1.0:  # Show info under the threshold
                update_annot({"ind": [nearest_index]})
                fig.canvas.draw_idle()
            else:
                annot.set_visible(False)
                fig.canvas.draw_idle()

    fig.canvas.mpl_connect("motion_notify_event", on_mouse_move)
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Satellite Number')
    parser.add_argument('input_folder', help='path to db3 folder')
    args = parser.parse_args()

    # Iterate through all db3 files in a folder
    input_folder = args.input_folder
    db3_files = [f for f in os.listdir(input_folder) if f.endswith('.db3')]

    if not db3_files:
        print("No .db3 files found in the specified folder.")
    else:
        for db3_file in db3_files:
            print(f"Processing {db3_file}...")
            full_path = os.path.join(input_folder, db3_file)
            process_bag_file(full_path)
        plot_data()
