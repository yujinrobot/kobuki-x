#
# License: BSD
#   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
#

import threading
import rospy
import actionlib
import dynamic_reconfigure.client
from yocs_navigator import BasicMoveController
import std_msgs.msg as std_msgs
import vending_machine_msgs.msg as vending_machine_msgs

VM_INTERACTOR_ACTION_NAME = 'vm_interactor'
VM_ORDER_RESULT     = 'drink_order_result'
VM_DRINK_ORDER            = 'drink_order'

ENABLE_AR_PAIR_APPROACH = 'enable_ar_pair_approach'
RESULT_AR_PAIR_APPROACH = 'result_ar_pair_approach'

class VendingMachineInteractor(object):
    """
    Provides simple action to interact with vending machine.
    """

    _vm_interactor_action_name = 'vm_interactor'

    def __init__(self):
        
        self._thread = None
        self._wait_for_drink_timeout = 10.0

        self._init_params()
        self._init_ros_handles()

        self._basic_move_controller = BasicMoveController()
        self._vending_machine_target_frame = self._create_target_frame()
        self._client = dynamic_reconfigure.client.Client(rospy.get_param('~pose_tracker', 'ar_track_alvar'))
        self._as_vm_interactor = actionlib.SimpleActionServer(self._vm_interactor_action_name, vending_machine_msgs.InteractorAction, auto_start=False)
        self._as_vm_interactor.register_goal_callback(self._process_interactor_goal)
        self._as_vm_interactor.register_preempt_callback(self._process_interactor_preempt)

    def _init_params(self):
        param = {}
        param['vending_machine_ar_id'] = rospy.get_param('~vending_machine_ar_id')
        param['ar_pair_global_prefix'] = rospy.get_param('ar_pair/global_prefix', 'global_marker')
        param['ar_pair_target_postfix'] = rospy.get_param('ar_pair/target_postfix', 'target')
        self._param = param

    def _init_ros_handles(self):
        self._pub = {}
        self._sub = {}

        # VM controller
        self._sub[VM_ORDER_RESULT] = rospy.Subscriber(VM_ORDER_RESULT, std_msgs.Int8, self._process_vm_order_result)
        self._pub[VM_DRINK_ORDER] = rospy.Publisher(VM_DRINK_ORDER, std_msgs.Int8, queue_size=2)

        # AR Pair Approach
        self._pub[ENABLE_AR_PAIR_APPROACH] = rospy.Publisher(ENABLE_AR_PAIR_APPROACH, std_msgs.String, queue_size=2)
        self._sub[RESULT_AR_PAIR_APPROACH] = rospy.Subscriber(RESULT_AR_PAIR_APPROACH, std_msgs.Bool, self._process_ar_pair_approach_result)

    def _create_target_frame(self):
        vm_id = self._param['vending_machine_ar_id']
        prefix = self._param['ar_pair_global_prefix']
        postfix = self._param['ar_pair_target_postfix']

        frame = str('%s_%s_%s'%(prefix, vm_id, postfix))

        return frame

    def _process_vm_order_result(self, msg):
        self._drink_received = True

    def _process_interactor_goal(self):
        goal = self._as_vm_interactor.accept_new_goal()
        self.loginfo("Received VM interactor goal %s"%str(goal))

        if goal.command == goal.APPROACH_VM:
            self._thread = threading.Thread(target=self._approach_vm)
            self._thread.start()
        elif goal.command == goal.ORDER_DRINK:
            self.loginfo(str(goal.drinks))
            self._thread = threading.Thread(target=self._order_drink, args=(goal.drinks,))
            self._thread.start()
        elif goal.command == goal.BACK_FROM_VM:
            self._thread = threading.Thread(target=self._back_from_vm)
            self._thread.start()
        else:
            message = "Invalid Command %s"%str(goal.command) 
            self._send_result(False, message)
    
    def _process_interactor_preempt(self): 
        self.logwarn('Received Preempt Request')

    def _process_ar_pair_approach_result(self, msg):
        self._ar_pair_approach_result = msg

    def _approach_vm(self):
        self._update_tracker(True)
        self._ar_pair_approach_result = None
        self.loginfo("target frame : %s"%self._vending_machine_target_frame)
        self._pub[ENABLE_AR_PAIR_APPROACH].publish(self._vending_machine_target_frame)

        while not self._ar_pair_approach_result and not rospy.is_shutdown():
            rospy.sleep(0.1)
        self._update_tracker(False)

        if self._ar_pair_approach_result.data:
            self._send_result(True, "Success")
        else:
            self._send_result(False, "Failed")

    def _back_from_vm(self):
        self.loginfo('Move back...')
        self._basic_move_controller.backward(0.4)
        self._send_result(True, "Success")

    def _order_drink(self, drinks):
        self.loginfo('Order Drink %s'%str(drinks))

        success = True 

        if type(drinks) is list or type(drinks) is tuple:
            for d in drinks:
                #self.loginfo("Ordering %s"%d)
                #rospy.sleep(2.0)
                self._drink_received = False
                self._pub[VM_DRINK_ORDER].publish(d)
                success = self._wait_for_drink()
        else:
            #self.loginfo("Ordering %s"%drinks)
            #rospy.sleep(2.0)
            self._drink_received = False
            self._pub[VM_DRINK_ORDER].publish(drinks)
            success = self._wait_for_drink()

        if success:
            self._send_result(True, "Success")
        else:
            self._send_result(False, "Timeout!")


    def _wait_for_drink(self):
        t1 = rospy.Time.now()

        while not self._drink_received and not rospy.is_shutdown():
            t2 = rospy.Time.now()

            #if (t2 - t1) > rospy.Duration.from_sec(self._wait_for_drink_timeout):
            #    self.loginfo("t2 - t1 : %s"%(t2-t1))
            #    self.loginfo(rospy.Duration.from_sec(self._wait_for_drink_timeout))
            #    return False
            rospy.sleep(0.1)

        return True


    def _send_result(self, success, message):
        if success:
            self.loginfo(message)
        else:
            self.logwarn(message)

        r = vending_machine_msgs.InteractorResult()
        r.success = success
        r.message = message
        self._as_vm_interactor.set_succeeded(r)

    def _update_tracker(self, enabled):
        params = { 'enabled' : enabled}
        config = self._client.update_configuration(params)

    def loginfo(self, msg):
        rospy.loginfo('VM Interactor : %s'%str(msg))

    def logwarn(self, msg):
        rospy.logwarn('VM Interactor : %s'%str(msg))

    def spin(self):
        self.loginfo("VM Interactor : Ready for ID[%s]"%(self._param['vending_machine_ar_id']))
        self._as_vm_interactor.start()
        rospy.spin()
