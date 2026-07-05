"""Icecovery Air Quality Monitor.

Endpoint layout (must match the firmware):
  ep 1:  Temperature/Humidity/CO2/Time (SCD41 + Zigbee mesh) -> native sensors
  ep 3:  AnalogInput present_value            -> sensor (TVOC, ppb)
  ep 6:  AnalogInput present_value            -> sensor (recovery-reboot count, diag)
  ep 7:  AnalogInput present_value            -> sensor (last reset reason, diag)
  ep 8:  custom cluster 0xFC10                -> button (recalibrate TVOC baseline)
  ep 9:  custom cluster 0xFC11                -> button (force refresh display)
  ep 10: AnalogOutput present_value           -> native number (SCD41 temp offset, C)
  ep 11: AnalogOutput present_value           -> native number (SCD41 altitude, m)

eCO2/CH2O/AQI (old EP2/4/5) were removed: the module's firmware only derives
those from its stuck internal baseline (see docs/re-report.md), so the firmware
no longer exposes them. TVOC stays on EP3 (host-computed from the raw MOX ADC),
keeping its HA entity history. The old EP12 (display timezone offset) was removed
too: the firmware now pulls already-timezone/DST-adjusted local time straight from
the coordinator's Time cluster (LocalTime), so no manual offset is needed.

EP10/11 are plain AnalogOutput clusters -- ZHA maps those to a native "number"
entity on its own (named/ranged/united from the firmware cluster attributes), so
this quirk doesn't declare them. It only moves them from the default "Controls"
section into "Config" via `.change_entity_metadata(new_entity_category=...)`, which
edits the native entity in place (no duplicate, firmware name/range/unit kept). Do
NOT re-declare them as quirk numbers to recategorize -- a plain quirk-v2 number on
a standard cluster does not suppress native discovery, so you'd get a duplicate.
(change_entity_metadata needs a recent zha-device-handlers; if HA logs "QuirkBuilder
has no attribute change_entity_metadata" the installed zhaquirks is too old and the
numbers stay under "Controls".) The write-only custom-cluster buttons still need
their quirk entries.

Drop this file into your configured zha custom-quirks directory and re-interview
the device after the endpoint change.
"""

import zigpy.types as t
from zigpy.quirks import CustomCluster
from zigpy.quirks.v2 import (
    QuirkBuilder,
    ReportingConfig,
    SensorDeviceClass,
    SensorStateClass,
)
from zigpy.quirks.v2.homeassistant import EntityType
from zigpy.zcl.clusters.general import AnalogInput, AnalogOutput
from zigpy.zcl.foundation import BaseAttributeDefs, ZCLAttributeDef


class RecalibrateTvocCluster(CustomCluster):
    """Manufacturer-specific cluster behind the recalibrate-TVOC button.

    Write-only by design: any write to ``trigger`` makes the firmware re-seed the
    TVOC clean-air baseline. (Custom clusters don't read back reliably under ZHA,
    but writes land fine -- which is all a button needs.)
    """

    cluster_id = 0xFC10
    name = "Recalibrate TVOC"
    ep_attribute = "recalibrate_tvoc"

    class AttributeDefs(BaseAttributeDefs):
        trigger = ZCLAttributeDef(id=0x0000, type=t.uint8_t, access="rw")


class DisplayRefreshCluster(CustomCluster):
    """Manufacturer-specific cluster behind the force-display-refresh button.

    Same write-only idiom as RecalibrateTvocCluster: any write to ``trigger`` makes
    the firmware redraw the e-paper display immediately and rebase its 10-minute
    auto-refresh timer from that moment.
    """

    cluster_id = 0xFC11
    name = "Force Refresh Display"
    ep_attribute = "force_refresh_display"

    class AttributeDefs(BaseAttributeDefs):
        trigger = ZCLAttributeDef(id=0x0000, type=t.uint8_t, access="rw")


(
    QuirkBuilder("Icecovery", "Air Quality Monitor")
    .sensor(
        endpoint_id=3,
        cluster_id=AnalogInput.cluster_id,
        attribute_name=AnalogInput.AttributeDefs.present_value.name,
        suggested_display_precision=0,
        device_class=SensorDeviceClass.VOLATILE_ORGANIC_COMPOUNDS_PARTS,
        state_class=SensorStateClass.MEASUREMENT,
        unit="ppb",
        reporting_config=ReportingConfig(min_interval=5, max_interval=60, reportable_change=1),
        translation_key="tvoc",
        fallback_name="Total Volatile Organic Compounds Concentration",
    )
    # Diagnostic sensors backing the firmware's connectivity watchdog: how many
    # times the device self-recovered (rebooted to rejoin) and the reason of the
    # last reset (esp_reset_reason enum: 1=power-on, 3=software/self-recovery,
    # 4=panic, 5-7=watchdog).
    .sensor(
        endpoint_id=6,
        cluster_id=AnalogInput.cluster_id,
        attribute_name=AnalogInput.AttributeDefs.present_value.name,
        suggested_display_precision=0,
        state_class=SensorStateClass.TOTAL_INCREASING,
        translation_key="recovery_reboots",
        fallback_name="Recovery reboots",
        entity_type=EntityType.DIAGNOSTIC,
    )
    .sensor(
        endpoint_id=7,
        cluster_id=AnalogInput.cluster_id,
        attribute_name=AnalogInput.AttributeDefs.present_value.name,
        suggested_display_precision=0,
        translation_key="last_reset_reason",
        fallback_name="Last reset reason",
        entity_type=EntityType.DIAGNOSTIC,
    )
    # Recalibrate-TVOC button: ZHA makes no native button for a custom cluster, so
    # this only exists with the quirk. Pressing it writes the custom cluster's
    # attribute; the firmware catches the write and re-seeds the TVOC baseline.
    .replaces(RecalibrateTvocCluster, endpoint_id=8)
    .write_attr_button(
        attribute_name="trigger",
        attribute_value=1,
        cluster_id=RecalibrateTvocCluster.cluster_id,
        endpoint_id=8,
        entity_type=EntityType.CONFIG,
        translation_key="recalibrate_tvoc",
        fallback_name="Recalibrate TVOC baseline",
    )
    # Force-refresh-display button: same write-only idiom as the recalibrate button.
    .replaces(DisplayRefreshCluster, endpoint_id=9)
    .write_attr_button(
        attribute_name="trigger",
        attribute_value=1,
        cluster_id=DisplayRefreshCluster.cluster_id,
        endpoint_id=9,
        entity_type=EntityType.CONFIG,
        translation_key="force_refresh_display",
        fallback_name="Force refresh display",
    )
    # Move the natively-discovered SCD41 calibration numbers (temp offset EP10,
    # altitude EP11) from "Controls" into "Config", in place -- see module docstring.
    .change_entity_metadata(
        endpoint_id=10, cluster_id=AnalogOutput.cluster_id,
        new_entity_category=EntityType.CONFIG,
    )
    .change_entity_metadata(
        endpoint_id=11, cluster_id=AnalogOutput.cluster_id,
        new_entity_category=EntityType.CONFIG,
    )
    .add_to_registry()
)
