"""Event-aware information aging policies."""

from dataclasses import dataclass
from enum import Enum


class Freshness(str, Enum):
    FRESH = "FRESH"
    AGING = "AGING"
    STALE = "STALE"
    EXPIRED = "EXPIRED"


@dataclass(frozen=True)
class AgingPolicy:
    fresh_s: float
    stale_s: float
    ttl_s: float


# Protocol event types: 0=NAV, 1=VICTIM, 2=GAS_HAZARD, 3=BLOCKED_PATH, 4=FIRE.
POLICIES: dict[int, AgingPolicy] = {
    0: AgingPolicy(300.0, 900.0, 1800.0),
    1: AgingPolicy(300.0, 900.0, 3600.0),
    2: AgingPolicy(120.0, 600.0, 1800.0),
    # 65535 seconds is the longest TTL representable by protocol v1.
    3: AgingPolicy(900.0, 3600.0, 65535.0),
    # Fire spreads quickly, so its memory ages fastest of all hazards.
    4: AgingPolicy(60.0, 300.0, 900.0),
}


def policy_for(event_type: int) -> AgingPolicy:
    return POLICIES.get(int(event_type), POLICIES[0])


def ttl_for(event_type: int) -> int:
    return int(policy_for(event_type).ttl_s)


def freshness(
    age_s: float,
    ttl_s: float | None = None,
    fresh_s: float | None = None,
    aging_s: float | None = None,
    event_type: int = 0,
) -> Freshness:
    """Classify age with an event policy, bounded by the encoded packet TTL."""
    policy = policy_for(event_type)
    fresh_limit = policy.fresh_s if fresh_s is None else float(fresh_s)
    stale_limit = policy.stale_s if aging_s is None else float(aging_s)
    ttl_limit = policy.ttl_s if ttl_s is None else min(float(ttl_s), policy.ttl_s)
    age = max(0.0, float(age_s))
    if age >= ttl_limit:
        return Freshness.EXPIRED
    if age < fresh_limit:
        return Freshness.FRESH
    if age < stale_limit:
        return Freshness.AGING
    return Freshness.STALE
