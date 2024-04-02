import { ESP_ROM_BAUD, Logger } from "./const";
import { ESPLoader } from "./esp_loader";

export type { Logger } from "./const";
export { ESPLoader } from "./esp_loader";

export {
  CHIP_FAMILY_ESP32,
  CHIP_FAMILY_ESP32S2,
  CHIP_FAMILY_ESP32S3,
  CHIP_FAMILY_ESP8266,
  CHIP_FAMILY_ESP32C3,
  CHIP_FAMILY_ESP32C6,
  CHIP_FAMILY_ESP32H2,
} from "./const";

export const connect = async (logger: Logger) => {
  // - Request a port and open a connection.
  const port = await navigator.serial.requestPort();

  logger.log("Connecting...");
  await port.open({ baudRate: ESP_ROM_BAUD });

  logger.log("Connected successfully.");

  return new ESPLoader(port, logger);
};
