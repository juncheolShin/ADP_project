# bt_transitions.py

from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, List, Optional, TYPE_CHECKING

from state_machine.state_types import StateType

if TYPE_CHECKING:
    from state_machine.state_machine import StateMachine


# === BT node definitions =======================================================

class BTNode:
    """Base class for all behavior tree nodes."""
    def tick(self, sm: "StateMachine") -> Optional[StateType]:
        raise NotImplementedError


@dataclass
class ConditionNode(BTNode):
    cond: Callable[["StateMachine"], bool]

    def tick(self, sm: "StateMachine") -> Optional[StateType]:
        return None if self.cond(sm) else "FAIL"  # "FAIL"은 내부용 sentinel


@dataclass
class ActionNode(BTNode):
    action: Callable[["StateMachine"], StateType]

    def tick(self, sm: "StateMachine") -> Optional[StateType]:
        return self.action(sm)


@dataclass
class SequenceNode(BTNode):
    children: List[BTNode]

    def tick(self, sm: "StateMachine") -> Optional[StateType]:
        for child in self.children:
            result = child.tick(sm)
            # Condition 실패
            if result == "FAIL":
                return "FAIL"
            # Action 이 state 를 리턴하면 바로 반환
            if isinstance(result, StateType):
                return result
        # 모든 조건 통과했는데 Action이 없으면 실패 취급
        return "FAIL"


@dataclass
class SelectorNode(BTNode):
    children: List[BTNode]

    def tick(self, sm: "StateMachine") -> Optional[StateType]:
        for child in self.children:
            result = child.tick(sm)
            # 실패가 아니면 (None or StateType) 바로 반환
            if result != "FAIL":
                return result
        return "FAIL"


# === Leaf condition helpers (기존 _check_* 재사용) ===========================

def cond_only_ftg_zone(sm: "StateMachine") -> bool:
    return sm._check_only_ftg_zone

def cond_ftg_timer(sm: "StateMachine") -> bool:
    # TRAILING에서 느리게 너무 오래 가고 있는지
    return sm._check_ftg

def cond_enemy_in_front(sm: "StateMachine") -> bool:
    return sm._check_enemy_in_front

def cond_gb_free(sm: "StateMachine") -> bool:
    return sm._check_gbfree

def cond_ot_sector(sm: "StateMachine") -> bool:
    return sm._check_ot_sector

def cond_ot_lane_free(sm: "StateMachine") -> bool:
    return sm._check_ofree

def cond_splini_available(sm: "StateMachine") -> bool:
    return sm._check_availability_splini_wpts

def cond_close_to_raceline(sm: "StateMachine") -> bool:
    return sm._check_close_to_raceline


# === Action helpers ============================================================

def act_ftgonly(sm: "StateMachine") -> StateType:
    # FTGONLY 로 진입 시 카운터 리셋
    sm.trailing_to_gbtrack_count = 0
    return StateType.FTGONLY


def act_trailing(sm: "StateMachine") -> StateType:
    # TRAILING 로 갈 때는 gbtrack 카운터 리셋
    sm.trailing_to_gbtrack_count = 0
    return StateType.TRAILING


def act_overtake(sm: "StateMachine") -> StateType:
    # 오버테이크 진입 시 카운터 리셋
    sm.trailing_to_gbtrack_count = 0
    return StateType.OVERTAKE


def act_gb_track_with_hysteresis(sm: "StateMachine") -> StateType:
    """
    GB_TRACK 로 바로 점프하지 않고, 이전 상태/카운터를 보고
    TRAILING_TO_GBTRACK -> GB_TRACK 로 점진적으로 전환.
    """
    # 현재 GB_TRACK 이면 그대로 유지
    if sm.state == StateType.GB_TRACK:
        sm.trailing_to_gbtrack_count = 0
        return StateType.GB_TRACK

    # GB_TRACK 으로 합류하는 과정: 카운터 기반
    sm.trailing_to_gbtrack_count += 1
    threshold = sm.trailing_to_gbtrack_counting_threshold

    if sm.trailing_to_gbtrack_count >= threshold:
        sm.trailing_to_gbtrack_count = 0
        return StateType.GB_TRACK
    else:
        # 아직 threshold 안 됐으면 "TRAILING_TO_GBTRACK" 를 유지/진입
        return StateType.TRAILING_TO_GBTRACK


def act_keep_current(sm: "StateMachine") -> StateType:
    """
    어떤 브랜치에도 안 걸리면 현재 state 유지 (fallback).
    """
    return sm.state


# === Behavior Tree 구성 ========================================================

def build_head_to_head_bt_root() -> BTNode:
    """
    head_to_head 모드에서 사용할 Behavior Tree 루트 노드를 구성.
    대략적인 의사 구조:

    Root (Selector)
      1) Only-FTG-Zone?
           -> FTGONLY
      2) FTG 타이머 만료?
           -> FTGONLY
      3) 상대/장애물 있음?
           3-1) 오버테이크 가능?
                   -> OVERTAKE
           3-2) GB path blocked?
                   -> TRAILING
           3-3) GB path free & raceline 근처?
                   -> TRAILING_TO_GBTRACK / GB_TRACK (히스테리시스)
           3-4) fallback -> TRAILING
      4) 상대 없음?
           4-1) GB free & raceline 근처?
                   -> GB_TRACK (히스테리시스)
           4-2) fallback -> TRAILING
      5) (마지막) 현재 상태 유지
    """

    # 1) FTG 전용 존이면 무조건 FTGONLY
    only_ftg_branch = SequenceNode([
        ConditionNode(cond_only_ftg_zone),
        ActionNode(act_ftgonly),
    ])

    # 2) TRAILING 에서 너무 느리게 오래 가면 FTGONLY
    ftg_timer_branch = SequenceNode([
        ConditionNode(cond_ftg_timer),
        ActionNode(act_ftgonly),
    ])

    # 3-x) 오버테이크 가능? (want to overtake?)
    overtake_possible_seq = SequenceNode([
        ConditionNode(cond_enemy_in_front),      # 상대가 있고
        ConditionNode(lambda sm: not cond_gb_free(sm)),  # GB 상에 장애물이 있어서 기본 라인이 막혀 있고
        ConditionNode(cond_ot_sector),           # OT 섹터이고
        ConditionNode(cond_splini_available),    # spline 유효하고
        ConditionNode(cond_ot_lane_free),        # OT 라인도 장애물 없음
        ActionNode(act_overtake),                # -> OVERTAKE
    ])

    # 3-y) 기본 라인이 막혀 있으면 TRAILING
    trailing_blocked_seq = SequenceNode([
        ConditionNode(cond_enemy_in_front),
        ConditionNode(lambda sm: not cond_gb_free(sm)),
        ActionNode(act_trailing),
    ])

    # 3-z) GB free + raceline 근처면 GB_TRACK 쪽으로 합류 (TRAILING_TO_GBTRACK 포함)
    trailing_to_gb_seq = SequenceNode([
        ConditionNode(cond_enemy_in_front),
        ConditionNode(cond_gb_free),
        ConditionNode(cond_close_to_raceline),
        ActionNode(act_gb_track_with_hysteresis),
    ])

    # enemy_in_front 이긴 한데 위 분기에 안 걸리면 그냥 TRAILING 유지
    enemy_fallback = SequenceNode([
        ConditionNode(cond_enemy_in_front),
        ActionNode(act_trailing),
    ])

    enemy_branch = SelectorNode([
        overtake_possible_seq,
        trailing_blocked_seq,
        trailing_to_gb_seq,
        enemy_fallback,
    ])

    # 4) 상대 없음인 경우
    no_enemy_gbtrack_seq = SequenceNode([
        ConditionNode(lambda sm: not cond_enemy_in_front(sm)),
        ConditionNode(cond_gb_free),
        ConditionNode(cond_close_to_raceline),
        ActionNode(act_gb_track_with_hysteresis),
    ])

    no_enemy_fallback = SequenceNode([
        ConditionNode(lambda sm: not cond_enemy_in_front(sm)),
        ActionNode(act_trailing),  # or GB_TRACK 유지 등 취향에 따라 조정 가능
    ])

    no_enemy_branch = SelectorNode([
        no_enemy_gbtrack_seq,
        no_enemy_fallback,
    ])

    # 최상위 루트
    root = SelectorNode([
        only_ftg_branch,         # 1) FTG only 구역
        ftg_timer_branch,        # 2) 속도 느린 상태에서 FTG 타이머 만료
        enemy_branch,            # 3) 상대/장애물 있는 경우
        no_enemy_branch,         # 4) 상대 없는 경우
        ActionNode(act_keep_current),  # 5) 아무 브랜치도 안 먹히면 현재 상태 유지
    ])

    return root


# Behavior Tree 루트는 한 번만 만들어서 재사용
_HEAD_TO_HEAD_BT_ROOT: Optional[BTNode] = None


def behavior_tree_transition(state_machine: "StateMachine") -> StateType:
    """
    head_to_head 모드에서 사용할 BT 기반 상태 전이 함수.
    기존 head_to_head_transition 대신 이 함수를 사용하면 됨.
    """
    global _HEAD_TO_HEAD_BT_ROOT
    if _HEAD_TO_HEAD_BT_ROOT is None:
        _HEAD_TO_HEAD_BT_ROOT = build_head_to_head_bt_root()

    result = _HEAD_TO_HEAD_BT_ROOT.tick(state_machine)

    # tick 결과가 "FAIL"이거나 None 이면 fallback 으로 현재 상태 유지
    if not isinstance(result, StateType):
        return state_machine.state
    return result

