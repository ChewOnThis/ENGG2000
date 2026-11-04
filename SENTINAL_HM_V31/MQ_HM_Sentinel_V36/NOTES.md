# V36: two-second startup

BOOT_WAIT_MS changes from 500 to 2000 ms. The motor remains in its existing BOOT brake state during the wait. Existing A / next battery boot workflow retained. V35 motor control and firing gates retained.

The operator reports a good run with persistent on-centre jitter preventing firing. The visible disconnected Serial Monitor currently retains only startup/idle output, not the reported EEPROM dump. No new firing-gate changes or causal claims are made without that dump. Reconnect and send D to retrieve it before arming another run.

Classic Nano old-bootloader compile checked. Not uploaded; startup timing not physically tested.
