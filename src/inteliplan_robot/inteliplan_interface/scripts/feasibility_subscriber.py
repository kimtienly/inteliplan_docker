'''
This is the feasibility module to verify the robot ability to reach an end-effector.
**to be updated**
'''

import rospy

class FeasibilitySubscriber:
    def __init__(self):
        rospy.loginfo("FeasibilitySubscriber -- initialised")
        pass
    def get_score(self, goal):
        if goal is None:
            return str(0)
        return str(1)

if __name__=='__main__':
    rospy.init_node('feasibility_subscriber')
    detection_sub = FeasibilitySubscriber()
    rospy.spin()