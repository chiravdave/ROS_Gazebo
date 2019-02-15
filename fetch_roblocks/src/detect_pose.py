import rospy
import ar_track_alvar_msgs.msg

detected_poses = None
sample_count = 10


def onARMarkerDetected(data,args):
	global detected_poses
	filter_id = args[0]
	for marker in data.markers:
		if(marker.id == filter_id):
			pos = marker.pose.pose.position
			ori = marker.pose.pose.orientation
			detected_poses.append([pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w])

def detectARMarkerTransform(marker_id):
	"""Takes sample_count samples of a marker and returns the average"""
	global detected_poses
	global sample_count

	detected_poses = []

	pose_subscriber = rospy.Subscriber("ar_pose_marker",ar_track_alvar_msgs.msg.AlvarMarkers,onARMarkerDetected,(marker_id))
	
	while len(detected_poses) < sample_count:
		pass
	pose_subscriber.unregister()

	pose = [np.average(row) for row in np.transpose(detected_poses)]
	return pose