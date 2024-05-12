import rospy


def controlCalculation():
    


if __name__ == '__main__':
    try:
        rospy.init_node('/control')
        rate = rospy.Rate(100)  # Publish rate of 10 Hz
        controlCalculation()
    except rospy.ROSInterruptException:
        pass
