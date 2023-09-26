# TI_Zstack_nodes

Here are some old codes based on Texas Instrument ZigBee stack, which is called ZStack, that we customized for our Smart Lighting modules.

## The SmartSense
This unit is designed to capture information from a set of sensors and report them through the ZigBee Cluster Library (ZCL) for any other node that wants to read.
List of sensors:
- PIR (occupancy detection)
- Humidity and Temperature (I2C sensor)
- Light (I2C sensor)
The node is battery-powered, so it joins the ZigBee network as an EndDevice to keep power consumption as low as possible.

## Smart Drive
This code belongs to a Constant Current LED Driver, which is controlled by a PWM signal. The node uses ZigBee Dimmable Light cluster and can be controlled by any other node in the network.
