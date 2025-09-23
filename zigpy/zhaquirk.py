"""Icecovery Air Quality Monitor."""

from zigpy.quirks.v2 import (
    QuirkBuilder,
    ReportingConfig,
    SensorDeviceClass,
    SensorStateClass,
)
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
    .add_to_registry()
)