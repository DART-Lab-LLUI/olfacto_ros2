import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn
import threading
import time

# --- FastAPI request model ---
class StimulusRequest(BaseModel):
    valve: int
    ratio: float
    duration: float
    total_flow: float

app = FastAPI()

# --- ROS Node Class ---
class OlfactometerController(Node):
    def __init__(self):
        super().__init__('olfactometer_controller')
        self.valve_pub = self.create_publisher(String, 'control_valve', 10)
        self.mfc0_pub = self.create_publisher(Float32, 'mfc0/set_flow_rate', 10)
        self.mfc1_pub = self.create_publisher(Float32, 'mfc1/set_flow_rate', 10)
        self.mfc2_pub = self.create_publisher(Float32, 'mfc2/set_flow_rate', 10)

        self.preload_delay = 2.0
        self.odr_boost = 2.0
        self.last_total_flow = 4.0  # initialize with a sensible default

        self.get_logger().info("Simplified olfactometer controller initialized.")

    def trigger_stimulus(self, valve, ratio, duration, total_flow):
        if valve < 0 or valve > 16 or not (0 <= ratio <= 1) or not (0 < total_flow <= 20):
            return {"status": "error", "message": "Invalid parameters."}

        self.last_total_flow = total_flow  # Store for post-stimulus reset

        thread = threading.Thread(
            target=self._stimulus_sequence,
            args=(valve, ratio, duration, total_flow),
            daemon=True
        )
        thread.start()
        return {"status": "started"}

    def _stimulus_sequence(self, valve, ratio, duration, total_flow):
        if valve != 0:
            # Odor preload with boosted flow
            self._open_valve(valve)
            boosted = total_flow * self.odr_boost
            self._set_flows(boosted * ratio, boosted * (1 - ratio), total_flow)
            self.get_logger().info(f"Preloading odor from valve {valve} for {self.preload_delay}s "
                                   f"with boosted flow (MFC0={boosted * ratio}, MFC1={boosted * (1 - ratio)})")
            time.sleep(self.preload_delay)

            # Deliver odor
            self._switch_3way(True)
            self._set_flows(total_flow * ratio, total_flow * (1 - ratio), total_flow)
            self.get_logger().info(f"Set target flows: MFC0={total_flow * ratio}, MFC1={total_flow * (1 - ratio)}, MFC2={total_flow}")
            time.sleep(duration)
            self._close_valve(valve)

        else:
            # Control preload with slight boost
            boosted = total_flow * 1.2
            self._set_flows(total_flow * ratio, total_flow * (1 - ratio), boosted)
            self.get_logger().info(f"Preloading control line for {self.last_total_flow}s "
                                   f"with boosted flow {boosted} LPM")
            time.sleep(self.preload_delay)

            # Deliver control air
            self._switch_3way(False)
            self._set_flows(0.0, total_flow, total_flow)
            time.sleep(duration)

        # Reset using last known total flow
        self._switch_3way(False)
        self._set_flows(0.0, self.last_total_flow, self.last_total_flow)
        self.get_logger().info(f"Reset MFC1 and MFC2 to last total flow: {self.last_total_flow} LPM")


    def _switch_3way(self, odor_on: bool):
        msg = String()
        msg.data = "17:ON" if odor_on else "17:OFF"
        self.valve_pub.publish(msg)
        self.get_logger().info(f"3-way valve -> {'ODOR' if odor_on else 'CONTROL'}")

    def _open_valve(self, valve):
        self.valve_pub.publish(String(data=f"{valve}:ON"))
        self.get_logger().info(f"Valve {valve} opened")

    def _close_valve(self, valve):
        self.valve_pub.publish(String(data=f"{valve}:OFF"))
        self.get_logger().info(f"Valve {valve} closed")

    def _set_flows(self, mfc0, mfc1, mfc2):
        self.mfc0_pub.publish(Float32(data=mfc0))
        self.mfc1_pub.publish(Float32(data=mfc1))
        self.mfc2_pub.publish(Float32(data=mfc2))

# --- FastAPI route ---
@app.post("/stimulus")
def stimulus_endpoint(req: StimulusRequest):
    return app.state.ros_node.trigger_stimulus(req.valve, req.ratio, req.duration, req.total_flow)

# --- Main ---
def main(args=None):
    rclpy.init(args=args)
    node = OlfactometerController()
    app.state.ros_node = node

    thread = threading.Thread(
        target=uvicorn.run,
        args=(app,),
        kwargs={"host": "0.0.0.0", "port": 8000},
        daemon=True
    )
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
