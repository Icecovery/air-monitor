"""Icecovery AQM device."""

from typing import Final

from zigpy.quirks import CustomCluster
from zigpy.quirks.v2 import (
    QuirkBuilder,
    ReportingConfig,
    SensorDeviceClass,
    SensorStateClass,
)
import zigpy.types as t
from zigpy.zcl.foundation import BaseAttributeDefs, ZCLAttributeDef, ZCLAttributeAccess
from zigpy.zcl.clusters.measurement import CarbonDioxideConcentration

class TVOC(CustomCluster):
    """tVOC (PPB) cluster."""

    cluster_id: t.uint16_t = 0xFC00
    name: str = "Total Volatile Organic Compounds Concentration"
    ep_attribute: str = "tvoc"

    class AttributeDefs(BaseAttributeDefs):
        """Attribute definitions."""

        measured_value: Final = ZCLAttributeDef(
            id=0x0000, type=t.Single, access=(ZCLAttributeAccess.Read | ZCLAttributeAccess.Report), is_manufacturer_specific=True
        )

class Formaldehyde(CustomCluster):
    """Formaldehyde Concentration cluster."""
    cluster_id: Final[t.uint16_t] = 0xFC01
    name: str = "Formaldehyde Concentration"
    ep_attribute: str = "formaldehyde"

    class AttributeDefs(BaseAttributeDefs):
        """Attribute definitions."""

        measured_value: Final = ZCLAttributeDef(
            id=0x0000, type=t.Single, access=(ZCLAttributeAccess.Read | ZCLAttributeAccess.Report), is_manufacturer_specific=True
        )


class AQI(CustomCluster):
    """Air Quality Index (AQI) cluster."""

    cluster_id: t.uint16_t = 0xFC02
    name: str = "Air Quality Index"
    ep_attribute: str = "aqi"

    class AttributeDefs(BaseAttributeDefs):
        """Attribute definitions."""

        measured_value: Final = ZCLAttributeDef(
            id=0x0000, type=t.Single, access=(ZCLAttributeAccess.Read | ZCLAttributeAccess.Report), is_manufacturer_specific=True
        )

class eCO2(CarbonDioxideConcentration, CustomCluster):
    """Estimated Carbon Dioxide (CO₂) cluster."""
    name: Final = "Estimated Carbon Dioxide (CO₂) Concentration"
    ep_attribute: Final = "estimated_carbon_dioxide_concentration"

(
    QuirkBuilder("Icecovery", "AQM")
    .applies_to("Icecovery", "AQM")
    .friendly_name(manufacturer="Icecovery", model="Air Quality Monitor")
    .replaces(TVOC)
    .replaces(Formaldehyde)
    .replaces(AQI)
    .replaces(eCO2, endpoint_id=2)
    .sensor(
        TVOC.AttributeDefs.measured_value.name,
        TVOC.cluster_id,
        endpoint_id=1,
        state_class=SensorStateClass.MEASUREMENT,
        device_class=SensorDeviceClass.VOLATILE_ORGANIC_COMPOUNDS_PARTS,
        reporting_config=ReportingConfig(
            min_interval=5, max_interval=60, reportable_change=1
        ),
        unit="ppb",
        translation_key="tvoc",
        fallback_name="TotalVolatileOrganicCompoundsConcentration",
    )
    .sensor(
        Formaldehyde.AttributeDefs.measured_value.name,
        Formaldehyde.cluster_id,
        endpoint_id=1,
        state_class=SensorStateClass.MEASUREMENT,
        device_class=SensorDeviceClass.VOLATILE_ORGANIC_COMPOUNDS_PARTS,
        reporting_config=ReportingConfig(
            min_interval=5, max_interval=60, reportable_change=1
        ),
        unit="ppb",
        translation_key="formaldehyde",
        fallback_name="Formaldehyde",
    )
    .sensor(
        AQI.AttributeDefs.measured_value.name,
        AQI.cluster_id,
        endpoint_id=1,
        state_class=SensorStateClass.MEASUREMENT,
        device_class=SensorDeviceClass.AQI,
        reporting_config=ReportingConfig(
            min_interval=5, max_interval=60, reportable_change=1
        ),
        translation_key="aqi",
        fallback_name="AirQualityIndex",
    )
    .sensor(
        eCO2.AttributeDefs.measured_value.name,
        eCO2.cluster_id,
        state_class=SensorStateClass.MEASUREMENT,
        device_class=SensorDeviceClass.CO2,
        reporting_config=ReportingConfig(
            min_interval=5, max_interval=60, reportable_change=0.000001
        ),
        unit="ppm",
        translation_key="eco2",
        fallback_name="EstimatedCarbonDioxideConcentration",
    )
    .add_to_registry()
)