"""Reference network profiles for known demo locations."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SiteNetworkProfile:
    """Static WAN settings that must be entered in the ipTIME setup tool."""

    name: str
    ip_address: str
    subnet_mask: str
    gateway: str
    dns: str
    secondary_dns: str


SITE_NETWORK_PROFILES: tuple[SiteNetworkProfile, ...] = (
    SiteNetworkProfile(
        name="220호",
        ip_address="113.198.84.249",
        subnet_mask="255.255.255.0",
        gateway="113.198.84.254",
        dns="113.198.74.100",
        secondary_dns="209.248.252.2",
    ),
    SiteNetworkProfile(
        name="시현장",
        ip_address="223.194.146.28",
        subnet_mask="255.255.255.0",
        gateway="223.194.146.254",
        dns="113.198.74.100",
        secondary_dns="203.248.252.2",
    ),
)


def format_site_network_profiles() -> list[str]:
    lines: list[str] = []
    for profile in SITE_NETWORK_PROFILES:
        lines.append(
            f"  {profile.name}: IP {profile.ip_address}, SN {profile.subnet_mask}, "
            f"GW {profile.gateway}, DNS {profile.dns}, secondary DNS {profile.secondary_dns}"
        )
    return lines
