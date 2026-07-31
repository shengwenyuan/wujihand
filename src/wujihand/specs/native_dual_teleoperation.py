"""Compatibility leaf for the validated NV-4 native transport."""

from __future__ import annotations

from .dual_teleoperation import (
    DualGlovePolicy,
    DualKinematicsPolicy,
    DualSupervisorPolicy,
    DualTeleoperationProfile,
    DualTrackerPolicy,
)


NATIVE_DUAL_TELEOPERATION_PROFILE_SCHEMA = (
    "wujihand.native_dual_teleoperation_profile.v1"
)
NATIVE_DUAL_TELEOPERATION_PROFILE_ID = (
    "isaac_nero_hand2_native_dual_teleoperation_v1"
)
NATIVE_DUAL_TELEOPERATION_PROFILE_STATUS = "simulation_live_pending_hil"
NATIVE_DUAL_TELEOPERATION_TRANSPORT_CONTRACT = (
    "wujihand.native_dual_teleoperation.v1"
)


class NativeDualTeleoperationProfile(DualTeleoperationProfile):
    """Parse the frozen NV-4 profile while sharing its policy vocabulary."""

    expected_schema = NATIVE_DUAL_TELEOPERATION_PROFILE_SCHEMA
    expected_profile_id = NATIVE_DUAL_TELEOPERATION_PROFILE_ID
    expected_status = NATIVE_DUAL_TELEOPERATION_PROFILE_STATUS
    expected_transport_contract = NATIVE_DUAL_TELEOPERATION_TRANSPORT_CONTRACT


# Stable compatibility names for callers of the validated NV-4 API.
NativeDualTrackerPolicy = DualTrackerPolicy
NativeDualKinematicsPolicy = DualKinematicsPolicy
NativeDualSupervisorPolicy = DualSupervisorPolicy
NativeDualGlovePolicy = DualGlovePolicy


__all__ = [
    "NATIVE_DUAL_TELEOPERATION_PROFILE_ID",
    "NATIVE_DUAL_TELEOPERATION_PROFILE_SCHEMA",
    "NATIVE_DUAL_TELEOPERATION_PROFILE_STATUS",
    "NATIVE_DUAL_TELEOPERATION_TRANSPORT_CONTRACT",
    "NativeDualGlovePolicy",
    "NativeDualKinematicsPolicy",
    "NativeDualSupervisorPolicy",
    "NativeDualTeleoperationProfile",
    "NativeDualTrackerPolicy",
]
