#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
#

import rospy
import yocs_vm_interactor

if __name__ == '__main__':
    rospy.init_node('vending_machine_interactor')
    interactor = yocs_vm_interactor.VendingMachineInteractor()
    interactor.loginfo('Initialized')
    interactor.spin()
    interactor.loginfo('Bye Bye')
