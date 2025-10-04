# Compatibility layer for dimos-unitree integration
# Re-exports go2_interfaces messages as unitree_go.msg

from go2_interfaces.msg import WebRtcReq, Go2State, IMU

# Make these available as unitree_go.msg imports
class msg:
    WebRtcReq = WebRtcReq
    Go2State = Go2State  
    IMU = IMU