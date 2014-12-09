#
# License: BSD
#   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
#

import threading
import rospy
import actionlib
import vending_machine_msgs.msg as vending_machine_msgs

VM_INTERACTOR_ACTION_NAME = 'vm_interactor'
VM_ORDER_RESULT     = 'drink_order_result'
VM_DRINK_ORDER            = 'drink_order'

class VendingMachineInteractor(object):
    """
    Provides simple action to interact with vending machine.
    """

    _vm_interactor_action_name = 'vm_interactor'

    def __init__(self):
        
        self._thread = None

        self._init_params()
        self._init_ros_handles()

        self._as_vm_interactor = actionlib.SimpleActionServer(self._vm_interactor_action_name, vending_machine_msgs.Interactor, auto_start=False)
        self._as_vm_interactor.register_goal_callback(self._process_interactor_goal)
        self._as_vm_interactor.register_preempt_callback(self._process_interactor_preempt)

    def _init_params(self):
        param = {}
        self.param = param

    def _init_ros_handles(self):
        # VM controller
        self._sub[VM_ORDER_RESULT] = rospy.Subscriber(VM_ORDER_RESULT, std_msgs.Int8, self._process_vm_order_result)
        self._pub[VM_DRINK_ORDER] = rospy.Publisher(VM_DRINK_ORDER, std_msgs.Int8, queue_size=2)

    def _process_vm_order_result(self, msg):
        pass

    def _process_interactor_goal(self, goal):
        goal = self._as_vm_interactor.accept_new_goal()
        self.loginfo("Received VM interactor goal %s"%str(goal))

        if goal.command == goal.APPROACH_VM:
            self._approach_vm()
        elif goal.command == goal.ORDER_DRINK:
            self._order_drink(goal.drinks)
        else:
            message = "Invalid Command %s"%str(goal.command) 
            self._send_result(False, message)
    
    def _process_interactor_preempt(self): 
        self.logwarn('Received Preempt Request')

    def _approach_vm(self):

    def _send_result(self, success, message):
        if success:
            self.loginfo(message)
        else:
            self.logwarn(message)

        r = vending_machine_msgs.InteractorResult()
        r.success = success
        r.message = message
        self._as_vm_interactor.set_succeeded(r)

    def loginfo(self, msg):
        rospy.loginfo('VM Interactor : %s'%str(msg))

    def logwarn(self, msg):
        rospy.logwarn('VM Interactor : %s'%str(msg))
