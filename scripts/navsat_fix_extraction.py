#! /usr/bin/python

import os, csv, enum, datetime
import rospy, roslib.packages
from sensor_msgs.msg import NavSatFix, NavSatStatus

file_save_path = rospy.get_param("file_save_path", default=os.path.join(roslib.packages.get_pkg_dir("data_analysis"), "doc"))
csv_header_list = ["Topic", "FrameID", "RosStamp", "Latitude", "Longitude", "Altitude", "NavSatStatus", "SatelliteSystem", "PositionCovarianceType", "PositionCovariance"]
fix_matrix = [[]]
fix_filtered_matrix = [[]]

class Status(enum.Enum):
    STATUS_NO_FIX = NavSatStatus.STATUS_NO_FIX
    STATUS_FIX = NavSatStatus.STATUS_FIX
    STATUS_SBAS_FIX = NavSatStatus.STATUS_SBAS_FIX
    STATUS_GBAS_FIX = NavSatStatus.STATUS_GBAS_FIX

class Service(enum.Enum):
    SERVICE_GPS = NavSatStatus.SERVICE_GPS
    SERVICE_GLONASS = NavSatStatus.SERVICE_GLONASS
    SERVICE_COMPASS = NavSatStatus.SERVICE_COMPASS
    SERVICE_GALILEO = NavSatStatus.SERVICE_GALILEO

class PositionCovarianceType(enum.Enum):
    COVARIANCE_TYPE_UNKNOWN = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    COVARIANCE_TYPE_APPROXIMATED = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    COVARIANCE_TYPE_DIAGONAL_KNOWN = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    COVARIANCE_TYPE_KNOWN = NavSatFix.COVARIANCE_TYPE_KNOWN

def fix_callback(msg = NavSatFix):

    navsat_status = navsat_service = pos_cov_type = ''
    for item in Status:
        if msg.status.status == item.value:
            navsat_status = item.name
    for item in Service:
        if msg.status.status == item.value:
            navsat_service = item.name
    for item in PositionCovarianceType:
        if msg.position_covariance_type == item.value:
            pos_cov_type = item.name
    
    pos_cov_list = [str(item) for item in msg.position_covariance] 
    pos_cov_row1 = ' ,'.join(pos_cov_list[0:3])
    pos_cov_row2 = ' ,'.join(pos_cov_list[3:6])
    pos_cov_row3 = ' ,'.join(pos_cov_list[6:])
    pos_cov_string = '[[{}], [{}], [{}]]'.format(pos_cov_row1, pos_cov_row2, pos_cov_row3)
    
    fix_matrix.append([
        fix_topic, msg.header.frame_id, msg.header.stamp.to_sec(), 
        msg.latitude, msg.longitude, msg.altitude, 
        navsat_status, navsat_service, pos_cov_type, pos_cov_string
    ])
    
    rospy.loginfo("Get Fix Data: {}".format(
        '[' + ', '.join([
            fix_topic, msg.header.frame_id, str(msg.header.stamp.to_sec()),
            str(msg.latitude), str(msg.longitude), str(msg.altitude)
        ]) + ']'
    ))


def filtered_fix_callback(msg = NavSatFix):
    
    navsat_status = navsat_service = pos_cov_type = ''
    for item in Status:
        if msg.status.status == item.value:
            navsat_status = item.name
    for item in Service:
        if msg.status.status == item.value:
            navsat_service = item.name
    for item in PositionCovarianceType:
        if msg.position_covariance_type == item.value:
            pos_cov_type = item.name
    
    pos_cov_list = [str(item) for item in msg.position_covariance] 
    pos_cov_row1 = ' ,'.join(pos_cov_list[0:3])
    pos_cov_row2 = ' ,'.join(pos_cov_list[3:6])
    pos_cov_row3 = ' ,'.join(pos_cov_list[6:])
    pos_cov_string = '[[{}], [{}], [{}]]'.format(pos_cov_row1, pos_cov_row2, pos_cov_row3)
    
    fix_filtered_matrix.append([
        fix_filtered_topic, msg.header.frame_id, msg.header.stamp.to_sec(), 
        msg.latitude, msg.longitude, msg.altitude, 
        navsat_status, navsat_service, pos_cov_type, pos_cov_string
    ])

    rospy.loginfo("Get Fix Filtered Data: {}".format(
        '[' + ', '.join([
            fix_filtered_topic, msg.header.frame_id, str(msg.header.stamp.to_sec()),
            str(msg.latitude), str(msg.longitude), str(msg.altitude)
        ]) + ']'
    ))
    
def shutdown_callback():

    fix_filtered_file_name = fix_filtered_topic.replace("/", "_") + "_{:%Y_%m_%d_%H_%M_%S}.csv".format(datetime.datetime.now())
    fix_file_name = fix_topic + "_{:%Y_%m_%d_%H_%M_%S}.csv".format(datetime.datetime.now())
    rospy.loginfo("NavSatFix Data is saved in [{}/{}]".format(file_save_path, fix_file_name))

    # Write Fix
    with open(os.path.join(file_save_path, fix_file_name), "w") as f_fix:
        fix_csv_writer = csv.writer(f_fix)
        fix_csv_writer.writerow(csv_header_list)
        for item in fix_matrix:
            if len(item) > 1:
                fix_csv_writer.writerow(item)

    rospy.loginfo("NavSatFixFiltered Data is saved in [{}/{}]".format(file_save_path, fix_filtered_file_name))
    
    # Write Fix Filtered
    with open(os.path.join(file_save_path, fix_filtered_file_name), "w") as f_fix_filtered:
        fix_filtered_csv_writer = csv.writer(f_fix_filtered)
        fix_filtered_csv_writer.writerow(csv_header_list)
        for item in fix_filtered_matrix:
            if len(item) > 1:
                fix_filtered_csv_writer.writerow(item)
            
if __name__ == '__main__':
    
    rospy.init_node(name="navsat_fix_extraction")
    
    global fix_topic, fix_filtered_topic
    fix_topic = rospy.get_param(param_name="fix_topic", default="fix")
    fix_filtered_topic = rospy.get_param(param_name="fix_filtered_topic", default="gps/filtered")
    
    rospy.Subscriber(fix_topic, NavSatFix, fix_callback, queue_size=30)
    rospy.Subscriber(fix_filtered_topic, NavSatFix, filtered_fix_callback, queue_size=30)
    rospy.loginfo("Ready to receive NavsatFix and NavsatFixFiltered from Topic.")
    rospy.spin()
    rospy.on_shutdown(shutdown_callback)
