# Package for Unitree Go2 robot integration with DimOS
# This package provides compatibility with dimos-unitree framework

from go2_interfaces.msg import WebRtcReq, Go2State, IMU

# Re-export for compatibility with dimos-unitree
__all__ = ['WebRtcReq', 'Go2State', 'IMU']