'''
----------------------------------------------------------------------------
This file is part of the Sanworks Pulse Pal repository
Copyright (C) 2022 Sanworks LLC, Rochester, New York, USA
----------------------------------------------------------------------------
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3.
This program is distributed  WITHOUT ANY WARRANTY and without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

from ArCOM_F2TTL import ArCOM
import numpy as np
import time

class Frame2TTLv2(object):
    def __init__(self, PortName):
        self.Port = ArCOM(PortName, 115200)
        self.Port.write(ord('C'), 'uint8')
        handshakeByte = self.Port.read(1, 'uint8')
        if handshakeByte != 218:
            raise F2TTLError('Error: Frame2TTL not detected on port ' + PortName + '.')
        self.Port.write(ord('#'), 'uint8')
        hardwareVersion = self.Port.read(1, 'uint8')
        if hardwareVersion != 2:
            raise F2TTLError('Error: Frame2TTLv2 requires hardware version 2.')
        self._lightThreshold = 100; # This is not a threshold of raw sensor data.
        self._darkThreshold = -150; # It is a 20-sample sliding window avg of
                                    # sample-wise differences in the raw signal.   
    @property
    def lightThreshold(self):
        return self._lightThreshold
    
    @lightThreshold.setter
    def lightThreshold(self, value):
        self.Port.write(ord('T'), 'uint8', (value, self.darkThreshold), 'int16');
        self._lightThreshold = value
    
    @property
    def darkThreshold(self):
        return self._darkThreshold
    
    @darkThreshold.setter
    def darkThreshold(self, value):
        self.Port.write(ord('T'), 'uint8', (self.lightThreshold, value), 'int16');
        self._darkThreshold = value
    
    def setLightThreshold_Auto(self): # Run with the sync patch set to black   
        self.Port.write(ord('L'), 'uint8')
        time.sleep(3)
        newThreshold = self.Port.read(1, 'int16')
        self.lightThreshold = newThreshold[0]
        
    def setDarkThreshold_Auto(self): # Run with the sync patch set to white 
        self.Port.write(ord('D'), 'uint8')
        time.sleep(3)
        newThreshold = self.Port.read(1, 'int16')
        self.darkThreshold = newThreshold[0]
        
    def read_sensor(self, nSamples): # Return contiguous samples (raw sensor data)
        self.Port.write(ord('V'), 'uint8', nSamples, 'uint32')
        value = self.Port.read(nSamples, 'uint16')
        return value
    
    def measure_photons(self, num_samples: int = 250) -> dict:
        """Measure <num_samples> values from the sensor and return basic stats.
        Mean, Std, SEM, Nsamples
        """
        sensorData = self.read_sensor(num_samples)
        out = {
            "mean_value": float(sensorData.mean()),
            "max_value": float(sensorData.max()),
            "min_value": float(sensorData.min()),
            "std_value": float(sensorData.std()),
            "sem_value": float(sensorData.std() / np.sqrt(num_samples)),
            "nsamples": float(num_samples),
        }
        return out
    
    def __repr__ (self): # Self description when the object is entered into the IPython console with no properties or methods specified
        return ('\nBpodHiFi with user properties:' + '\n\n'
        'Port: ArCOMObject(' + self.Port.serialObject.port + ')'  + '\n'
        'lightThreshold: ' + str(self.lightThreshold) + '\n'
        'darkThreshold: ' + str(self.darkThreshold) + '\n'
        )
    
    def __del__(self):
        self.Port.close()
        
class F2TTLError(Exception):
    pass