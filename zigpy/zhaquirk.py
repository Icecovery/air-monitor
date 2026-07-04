"""Icecovery Air Quality Monitor."""

from zigpy.quirks.v2 import (
    QuirkBuilder,
    ReportingConfig,
    SensorDeviceClass,
    SensorStateClass,
)
from zigpy.quirks.v2.homeassistant import EntityType
from zigpy.zcl.clusters.general import AnalogInput

(
    QuirkBuilder("Icecovery", "Air Quality Monitor")
    .sensor(
        endpoint_id=2,
        cluster_id=AnalogInput.cluster_id,
        attribute_name=AnalogInput.AttributeDefs.present_value.name,
        suggested_display_precision=0,
        device_class=SensorDeviceClass.CO2,
        state_class=SensorStateClass.MEASUREMENT,
        unit="ppm",
        reporting_config=ReportingConfig(min_interval=5, max_interval=60, reportable_change=1),
        translation_key="eco2",
        fallback_name="Estimated CO2 Concentration",
    )
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
    .sensor(
        endpoint_id=4,
        cluster_id=AnalogInput.cluster_id,
        attribute_name=AnalogInput.AttributeDefs.present_value.name,
        suggested_display_precision=0,
        device_class=SensorDeviceClass.VOLATILE_ORGANIC_COMPOUNDS_PARTS,
        state_class=SensorStateClass.MEASUREMENT,
        unit="ppb",
        reporting_config=ReportingConfig(min_interval=5, max_interval=60, reportable_change=1),
        translation_key="formaldehyde",
        fallback_name="Formaldehyde Concentration",
    )
    .sensor(
        endpoint_id=5,
        cluster_id=AnalogInput.cluster_id,
        attribute_name=AnalogInput.AttributeDefs.present_value.name,
        suggested_display_precision=0,
        device_class=SensorDeviceClass.AQI,
        state_class=SensorStateClass.MEASUREMENT,
        reporting_config=ReportingConfig(min_interval=5, max_interval=60, reportable_change=1),
        translation_key="aqi",
        fallback_name="Air Quality Index",
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
    .add_to_registry()
)