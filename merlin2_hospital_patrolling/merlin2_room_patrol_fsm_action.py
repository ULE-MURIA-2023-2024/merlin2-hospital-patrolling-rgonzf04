#!/usr/bin/python
# TODO: write the patrol FSM action

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from typing import List
from merlin2_hospital_patrolling.pddl import room_type, room_at, room_patrolled
from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at

from merlin2_fsm_action import (
    Merlin2FsmAction,
    Merlin2BasicStates
)

from yasmin import CbState
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin.blackboard import Blackboard

from kant_dto import (
    PddlObjectDto,
    PddlConditionEffectDto,
)

class Merlin2RoomPatrolFSMAction(Merlin2FsmAction):

    def __init__(self) -> None:

        self._room = PddlObjectDto(room_type, "room")
        self._wp = PddlObjectDto(wp_type, "wp")
        
        super().__init__("room_patrol")

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        tts_state = self.create_state(Merlin2BasicStates.TTS)

        self.add_state(
            "ROTATING",
            CbState([SUCCEED], self.rotate),
            transitions={
                SUCCEED: "PREPARING_TEXT"
            }
        )

        self.add_state(
            "PREPARING_TEXT",
            CbState([SUCCEED], self.prepare_text),
            transitions={
                SUCCEED: "SPEAKING"
            }
        )

        self.add_state(
            "SPEAKING",
            tts_state
        )

    def rotate(self, blackboard: Blackboard) -> str:
        # Create a Twist message to rotate the robot
        twist = Twist()
        twist.angular.z = 0.5 

        # Publish the message
        self.cmd_vel_publisher.publish(twist)

        # Set a duration for the rotation
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=10))

        # Stop the rotation
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

        blackboard.text = "Rotating"
        return SUCCEED

    def prepare_text(self, blackboard: Blackboard) -> str:
        # room_name = blackboard.merlin2_action_goal.objects[0][-1]
        # blackboard.text = f"Strecher room {room_name} patrolled"
        blackboard.text = f"Strecher room patrolled"
        return SUCCEED

    def create_parameters(self) -> List[PddlObjectDto]:
        return [self._room, self._wp]

    def create_conditions(self) -> List[PddlConditionEffectDto]:
        
        # cond_1 = PddlConditionEffectDto(
        #     room_patrolled,
        #     [self._room],
        #     PddlConditionEffectDto.AT_START,
        #     is_negative=False
        # )

        cond_2 = PddlConditionEffectDto(
            robot_at,
            [self._wp],
            PddlConditionEffectDto.AT_START
        )

        cond_3 = PddlConditionEffectDto(
            room_at,
            [self._room, self._wp],
            PddlConditionEffectDto.AT_START
        )

        return [cond_2, cond_3]

    def create_efects(self) -> List[PddlConditionEffectDto]:

        effect_1 = PddlConditionEffectDto(
            room_patrolled,
            [self._room],
            time=PddlConditionEffectDto.AT_END
        )

        return [effect_1]
    
def main():
    rclpy.init()
    node = Merlin2RoomPatrolFSMAction()
    node.join_spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()