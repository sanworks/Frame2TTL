Note: To upload the firmware correctly, you have to add the Sparkfun SAMD21 Mini board definition to your Arduino installation. Follow the instructions in this link:

https://learn.sparkfun.com/tutorials/samd21-minidev-breakout-hookup-guide/setting-up-arduino

Frame2TTL_Filtered firmware contains a experimental parameter to filter the light intensity signal with a sliding window average of configurable size (1-10 samples). Using a 10-sample filter, the TTL output signal *should* still follow screen intensity changes within <1ms. 