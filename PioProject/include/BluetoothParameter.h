#ifndef BLUETOOTH_PARAMETER_H
#define BLUETOOTH_PARAMETER_H

#include "WString.h"
#include "parameter.h"


static const char MAC_ADDR[] PROGMEM = "MAC address / iBeacon UUID";
static const char ALIAS   [] PROGMEM = "Alias";


class BluetoothParameter {
public:
  BluetoothParameter(uint8_t index) :
    bm_mac_addr      (alias_id+1,   MAC_ADDR,    "",   40),
    bm_mac_alias     (alias_id  ,   ALIAS   ,    "",   20),
    bm_mac_sep       (PSTR("<hr width=\"80%\" align=\"center\" noshade>"))
  {
    // Write the required ID's to retrieve preference data.
    sprintf (alias_id, "abm%02d", index); 
    // Alias starts at 'a', mac addr starts with 'bm'. This way the same buffer can be used for both!!
  }

  const char *getMacAddress() {
    return bm_mac_addr.getValue();
  }

  void setMacAddress(const char * val) {
    bm_mac_addr.setValue(val);
  }

  const char *getAlias() {
    return bm_mac_alias.getValue();
  }

  void setAlias(const char * val) {
    bm_mac_alias.setValue(val);
  }

private:
  /*const*/ char alias_id[8];

  Parameter bm_mac_addr;
  Parameter bm_mac_alias;
  Parameter bm_mac_sep;
};

#endif // BLUETOOTH_PARAMETER_H