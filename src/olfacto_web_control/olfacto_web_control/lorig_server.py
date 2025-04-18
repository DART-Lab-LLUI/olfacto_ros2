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


# --- Combined ROS Node + Web API ---
class OlfactometerController(Node):
    def __init__(self):
        super().__init__('olfactometer_controller_lorig')
        self.valve_pub = self.create_publisher(String, 'control_valve', 10)
        self.mfc0_pub = self.create_publisher(Float32, 'mfc0/set_flow_rate', 10)
        self.mfc1_pub = self.create_publisher(Float32, 'mfc1/set_flow_rate', 10)
        self.current_valve = None
        self.last_total_flow = 4.0
        self.delay_time = 0.2

        self.get_logger().info("Olfactometer Controller with HTTP interface initialized.")

    def trigger_stimulus(self, valve, ratio, duration, total_flow):
        if valve < 0 or valve > 16 or not (0 <= ratio <= 1.0) or not (0.0 <= total_flow <= 20.0):
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
        flow_mfc0 = total_flow * ratio
        flow_mfc1 = total_flow * (1 - ratio)

        # Close current valve if different
        if self.current_valve is not None and self.current_valve != valve:
            self._close_valve(self.current_valve)
            time.sleep(self.delay_time)

        # Open new valve if needed
        if valve != 0:
            self._open_valve(valve)
        else:
            self.get_logger().info("No valve selected (clean air flush)")

        # Set stimulus flow and sleep for duration
        self._set_flows(flow_mfc0, flow_mfc1)
        time.sleep(duration)

        # Reset after stimulus
        if valve != 0 and self.current_valve == valve:
            self._close_valve(valve)

        self._set_flows(0.0, total_flow)
        self.get_logger().info("Reset MFCs after stimulus.")


    def _open_valve(self, valve):
        self.valve_pub.publish(String(data=f"{valve}:ON"))
        self.current_valve = valve
        self.get_logger().info(f"Valve {valve} opened")

    def _close_valve(self, valve):
        self.valve_pub.publish(String(data=f"{valve}:OFF"))
        self.current_valve = None
        self.get_logger().info(f"Valve {valve} closed")

    def _set_flows(self, mfc0, mfc1):
        self.mfc0_pub.publish(Float32(data=mfc0))
        self.mfc1_pub.publish(Float32(data=mfc1))
        self.get_logger().info(f"Flow rates set to MFC0={mfc0}, MFC1={mfc1}")

# --- FastAPI App ---
app = FastAPI()

@app.post("/stimulus")
def stimulus_endpoint(req: StimulusRequest):
    ros_node = app.state.ros_node
    return ros_node.trigger_stimulus(req.valve, req.ratio, req.duration, req.total_flow)


# --- Main entry point ---
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
        node.get_logger().info("Shutting down. Closing last valve and set MFC's to 0.0")
        if node.current_valve:
            node._close_valve(node.current_valve)
        node._set_flows(0.0, 0.0)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
