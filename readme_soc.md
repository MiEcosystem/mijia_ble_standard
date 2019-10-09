SOCF in Proximity Reporter
======================================================================
#Overview
This application can be built with State of Charge Function (SOCF) support.

## SOCF feature

- SOCF feature can be enabled with below defines in custom\_config\_qspi.h.
	- #define dg\_configUSE_SOC                        1
- To check SOC value from serial output, below feature must be enabled.
	- #define DEBUG\_SOC

	It is recommended that this feature is disabled at release version because the device wakes up every sec for serial output.

## Required information for SOCF

- Battery profile
	- Battery profile data in projects\dk_apps\demos\pxp_reporter\config\ custom\_socf\_battery\_profile.h must be prepared.
	- vol\_dis\_low\_0 and vol\_dis\_high\_0 can be gained through battery profiling.
- Additional information
	- Below defines must be decided through battery profiling properly.
		- #define SOC\_BATT\_LOW\_CURRENT 1000
		- #define SOC\_BATT\_HIGH\_CURRENT 20000
		- #define SOCF\_BATTERY\_CAPACITANCE 120
		- #define SOCF\_CHARGING\_CURRENT 60
	 
## Report SOC to Host

- Any application can get SOC value through function soc\_get\_soc anytime. The range of the return value of soc\_get\_soc is 0~1000. 
- The value can be reported to host through BAS profile like below.
	- bas\_set\_level(bas, (soc\_get\_soc()+5)/10, true);
