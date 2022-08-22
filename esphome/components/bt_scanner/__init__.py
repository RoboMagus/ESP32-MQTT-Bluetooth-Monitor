import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_MODEL
from esphome.core import CORE
from esphome.components.esp32 import add_idf_sdkconfig_option


CODEOWNERS = ["@RoboMagus"]
# Likely fails with these, needs to be verified...
CONFLICTS_WITH = ["esp32_ble", "esp32_ble_tracker", "esp32_ble_beacon", "esp32_ble_server"]
DEPENDENCIES = ["esp32", "time"]

bt_scanner_ns = cg.esphome_ns.namespace("bt_scanner")
BtClassicScanner = bt_scanner_ns.class_("BluetoothClassicScanner", cg.Component)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BtClassicScanner),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.require_framework_version(
        esp32_arduino=cv.Version(2, 0, 3)
    ),
)


async def to_code(config):
    # Indicate to BtScanner src Esphome includes are required
    cg.add_define("ESPHOME_BT_CLASSIC_SCAN")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)


#    cg.add_platformio_option("framework", "espidf")
#    add_idf_sdkconfig_option("CONFIG_BT_ENABLED", True)
#    
#    add_idf_sdkconfig_option("CONFIG_PARTITION_TABLE_CUSTOM", True)
#    add_idf_sdkconfig_option(
#        "CONFIG_PARTITION_TABLE_CUSTOM_FILENAME", "partitions_custom.csv"
#    )
#    cg.add_platformio_option("board_build.partitions", "partitions_custom.csv")
#
#    cg.add_build_flag("-DUSE_ESP_IDF")
#    cg.add_build_flag("-DUSE_ESP32_FRAMEWORK_ESP_IDF")
#
#    cg.add_define("ESPHOME_BT_CLASSIC_SCAN")