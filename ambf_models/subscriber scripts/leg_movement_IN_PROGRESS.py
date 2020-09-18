#!/usr/bin/env python3

import rospy
import message_filters
from ambf_msgs.msg import RigidBodyState

packet = 0

fslt_x =  []
fslt_y =  []
bslt_x =  []
bslt_y =  []

fsls_x =  []
fsls_y =  []
bsls_x =  []
bsls_y =  []

fsrt_x =  []
fsrt_y =  []
bsrt_x =  []
bsrt_y =  []

fsrs_x =  []
fsrs_y =  []
bsrs_x =  []
bsrs_y =  []


def callback(fslt,bslt,fsls,bsls,fsrt,bsrt,fsrs,bsrs):



    #mag of x/y for movement


    #if packet == 10:


    print('FrontSensorLeftThigh force y: {}'.format(fslt.wrench.force.y))
    print('BackSensorLeftThigh force y: {}'.format(bslt.wrench.force.y))
    print('FrontSensorLeftShank force y: {}'.format(fsls.wrench.force.y))
    print('BackSensorLeftShank force y: {}'.format(bsls.wrench.force.y))

    print('FrontSensorRightThigh force y: {}'.format(fsrt.wrench.force.y))
    print('BackSensorRightThigh force y: {}'.format(bsrt.wrench.force.y))
    print('FrontSensorRightShank force y: {}'.format(fsrs.wrench.force.y))
    print('BackSensorRightShank force y: {}'.format(bsrs.wrench.force.y))


if __name__ == '__main__':
    try:
        rospy.init_node('leg_movement', anonymous=True)

        fslt_sub = message_filters.Subscriber("/ambf/env/FrontSensorLeftThigh/State", RigidBodyState)
        bslt_sub = message_filters.Subscriber("/ambf/env/BackSensorLeftThigh/State", RigidBodyState)

        fsls_sub = message_filters.Subscriber("/ambf/env/FrontSensorLeftShank/State", RigidBodyState)
        bsls_sub = message_filters.Subscriber("/ambf/env/BackSensorLeftShank/State", RigidBodyState)

        fsrt_sub = message_filters.Subscriber("/ambf/env/FrontSensorRightThigh/State", RigidBodyState)
        bsrt_sub = message_filters.Subscriber("/ambf/env/BackSensorRightThigh/State", RigidBodyState)

        fsrs_sub = message_filters.Subscriber("/ambf/env/FrontSensorRightShank/State", RigidBodyState)
        bsrs_sub = message_filters.Subscriber("/ambf/env/BackSensorRightShank/State", RigidBodyState)
        
        sub_list = [fslt_sub, bslt_sub, fsls_sub, bsls_sub, fsrt_sub, bsrt_sub, fsrs_sub, bsrs_sub]

        body = message_filters.TimeSynchronizer(sub_list, 1)
        body.registerCallback(callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass  
