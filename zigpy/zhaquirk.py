"""Icecovery Air Quality Monitor.

Endpoint layout (must match the firmware):
  ep 1: Temperature/Humidity/CO2 (SCD41)      -> native sensors
  ep 3: AnalogInput present_value             -> sensor (TVOC, ppb)
  ep 6: AnalogInput present_value             -> sensor (recovery-reboot count, diag)
  ep 7: AnalogInput present_value             -> sensor (last reset reason, diag)
  ep 8: custom cluster 0xFC10                 -> button (recalibrate TVOC baseline)

eCO2/CH2O/AQI (old EP2/4/5) were removed: the module's firmware only derives
those from its stuck internal baseline (see docs/re-report.md), so the firmware
no longer exposes them. TVOC stays on EP3 (host-computed from the raw MOX ADC),
keeping its HA entity history.

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
from zigpy.zcl.clusters.general import AnalogInput
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
    .add_to_registry()
)
