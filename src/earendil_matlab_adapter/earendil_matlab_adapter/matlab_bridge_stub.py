"""
matlab_bridge_stub.py

MATLAB northbound adapter — STUB.
MATLAB tarafından gönderilen komutları ROS 2 mesajlarına köprüler.

TODO (Faz 4): Gerçek MATLAB ROS Toolbox veya WebSocket bağlantısı
Simülasyon için: rosbridge_server WebSocket üzerinden MATLAB bağlantısı
"""


class MatlabBridgeStub:
    """
    MATLAB komutlarını ROS 2 mesajlarına çevirir.
    Şu an sadece stub — gerçek implementasyon ileride.
    """

    def receive_command(self, raw_data: dict) -> dict:
        """
        MATLAB'dan gelen ham veriyi RscpCommand formatına çevirir.
        TODO (Faz 4): Gerçek MATLAB veri formatı ile eşleştir
        """
        return {
            'command_type': raw_data.get('cmd', 0),
            'payload': raw_data.get('params', ''),
            'source': 'matlab',
        }

    def send_telemetry(self, mission_state: dict) -> None:
        """
        Rover telemetrisini MATLAB'a iletir.
        TODO (Faz 4): MATLAB tarafına gönder
        """
        pass
