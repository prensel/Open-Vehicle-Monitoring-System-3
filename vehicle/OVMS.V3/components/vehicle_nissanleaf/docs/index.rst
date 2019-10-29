===========
Nissan Leaf
===========

Vehicle Type: **NL**

This vehicle type supports the Nissan Leaf and Nissan e-NV200.

----------------
Support Overview
----------------

=========================== ==============
Function                    Support Status
=========================== ==============
Hardware                    Any OVMS v3 (or later) module. Vehicle support: 2011-2017 (24kWh & 30kWh)
Vehicle Cable               1779000 Nissan Leaf OBD-II to DB9 Data Cable for OVMS
GSM Antenna                 1000500 Open Vehicles OVMS GSM Antenna (or any compatible antenna)
GPS Antenna                 1020200 Universal GPS Antenna (SMA Connector) (or any compatible antenna)
SOC Display                 Yes (by default based on GIDS)
Range Display               Yes (by default based on GIDS)
GPS Location                Yes (from modem module GPS)
Speed Display               Yes (from vehicle speed PID)
Temperature Display         Yes (from vehicle temperature PIDs)
BMS v+t Display             Yes
TPMS Display                Yes (If hardware available)
Charge Status Display       Yes
Charge Interruption Alerts  Yes
Charge Control              Start charge only
Cabin Pre-heat/cool Control Yes* (see info below)
Lock/Unlock Vehicle         Not currently supported
Valet Mode Control          Not currently supported
Others
=========================== ==============

OVMS currently supports 2011-2017 Nissan LEAF and Nissan e-NV200

----------------------
Remote Climate Control
----------------------

^^^^^^^^^^^^^^^^
2011-2013 models
^^^^^^^^^^^^^^^^

Gen1 Cars (2011-2014) require a hardware modification to enable OVMS to control remote climate. Wire RC3 to TCU pin 11, `more info <https://carrott.org/emini/Nissan_Leaf_OVMS#Remote_Climate_Control)>`_

^^^^^^^^^^^^^^^^
2014-2016 models
^^^^^^^^^^^^^^^^

To use OVMS to activate remote climate the Nissan TCU (Telematics Control Unit) module must be unplugged if fitted (only on Acenta and Tekna models). The TCU is located behind the glovebox on LHD cars or on the right hand side of the drivers foot well on RHD cars. The large white plug on the rear of the TCU should be unplugged, push down tab in the middle and pull to unplug, `see video for RHD cars <https://photos.app.goo.gl/MuvpCaXQUjbCdoox6>`_ and `this page for LHD cars <http://www.arachnon.de/wb/pages/en/nissan-leaf/tcu.php>`_.

Note: Unplugging the TCU will disable Nissan EV connect / CARWINGS features e.g Nissan mobile app. All other car functions will not be effected e.g GPS, maps, radio, Bluetooth, microphone all work just the same as before. OVMS can be used to more than substitute the loss of Nissan Connect features. The TCU can be plugged back in at any point in the future if required.

OVMS remote climate support will 'just work' on LEAF Visia models and Visia/Acenta e-NV200 since these models do not have a TCU fitted.

Note: If you prefer not to unplug the Nissan TCU, all OVMS functions appart from remote climate will function just fine alongside the Nissan TCU.
 

^^^^^^^^^^^^^^^^
2016-2017 models
^^^^^^^^^^^^^^^^

**Remote climate control may not work on 2016-2017 cars.** This is because in 2016 Nissan moved the TCU from the EV CAN bus to the CAR CAN bus.

Set the model year as follows and if necessary configure 30 kWh model:

``config set xnl modelyear 2016``

or

``config set xnl modelyear 2017``

*Note: in latest OVMS fimware version model year and battery size can be set via the web config interface.*

^^^^^^^^^^^^^
30 kwh models
^^^^^^^^^^^^^

For models with a 30 kwhr battery pack, set  the capacity manually as follows:

``config set xnl maxGids 356``
``config set xnl newCarAh 79``

*Note: in latest OVMS firmware version model year and battery size can be set via the web config interface.*


^^^^^^^^^^^^
2018+ models
^^^^^^^^^^^^

2018+ 40kWh LEAF is not yet supported. Please get in touch if your interested in helping to add support. Relevant 2018 CANbus messages have already been decoded and documented, see `MyNissanLEAF thread <https://mynissanleaf.com/viewtopic.php?f=44&t=4131&start=480>`_.


-----------------
Range Calculation
-----------------

The OVMS uses two configuration options to calculate remaining range, whPerGid (default 80Wh/gid) and kmPerKWh (default 7.1km/kWh). The range calculation is based on the remaining gids reported by the LBC and at the moment does not hold 5% in reserve like LeafSpy. Feedback on this calculation is welcomed.

-----------------
Resources
-----------------

- Nissan LEAF supported added by Tom Parker, see `his wiki <https://carrott.org/emini/Nissan_Leaf_OVMS>`_ for lots of documentation and resources. Some info is outdated e.g climate control now turns off automatically.
- `MyNissanLEAF thread for Nissan CANbus decoding discussion <http://www.mynissanleaf.com/viewtopic.php?f=44&t=4131&hilit=open+CAN+discussion&start=440>`_

Assistance is appreciated as I haven't had time to try to override the TCU using the OVMS or find an alternative solution to prevent the TCU overriding the messages while still allowing the hands free microphone to work.
