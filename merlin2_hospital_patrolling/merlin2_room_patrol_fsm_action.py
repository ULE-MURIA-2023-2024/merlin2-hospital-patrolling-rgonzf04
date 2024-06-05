#!/usr/bin/python
# TODO: write the patrol FSM action

import rclpy
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
        
        super().__init__("room patrol")

        tts_state = self.create_state(Merlin2BasicStates.TTS)

        # self.add_state(
        #     "ROTATING",
        #     CbState([SUCCEED], self.rotate),
        #     transitions={
        #         SUCCEED: "PREPARING TEXT"
        #     }
        # )

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

    # def rotate (self, blackboard: Blackboard) -> str:
    #     blackboard.text = f"Rotating"
    #     return SUCCEED

    def prepare_text(self, blackboard: Blackboard) -> str:
        # room_name = blackboard.merlin2_action_goal.objects[0][-1]
        # blackboard.text = f"Strecher room {room_name} patrolled"
        blackboard.text = f"Strecher room patrolled"
        return SUCCEED

    def create_parameters(self) -> List[PddlObjectDto]:
        return [self._room, self._wp]

    def create_conditions(self) -> List[PddlConditionEffectDto]:
        
        cond_1 = PddlConditionEffectDto(
            room_patrolled,
            [self._room],
            PddlConditionEffectDto.AT_START,
            is_negative=False
        )

        cond_2 = PddlConditionEffectDto(
            robot_at,
            [self._wp],
            PddlConditionEffectDto.AT_START
        )

        cond_3 = PddlConditionEffectDto(
            room_at,
            [self.room, self._wp],
            PddlConditionEffectDto.AT_START
        )

        return [cond_1, cond_2, cond_3]

    def create_effects(self) -> List[PddlConditionEffectDto]:

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

if __name__ == "main":
    main()