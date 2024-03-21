import tf

def pose_msg_to_state(msg):
    '''
    Takes a msg of type odom and converts it to a 2D state vector
    '''
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    quaternion = [
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    ]
    _, _, theta = tf.transformations.euler_from_quaternion(quaternion)

    # Accessing linear velocities [v_x, v_y, w]
    v_x = msg.twist.twist.linear.x
    v_y = msg.twist.twist.linear.y
    w = msg.twist.twist.angular.z

    return [x,y,theta,v_x,v_y,w]