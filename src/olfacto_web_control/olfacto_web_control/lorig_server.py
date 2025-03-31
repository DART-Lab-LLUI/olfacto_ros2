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
        self.valve_publisher = self.create_publisher(String, 'control_valve', 10)
        self.mfc0_publisher = self.create_publisher(Float32, 'mfc0/set_flow_rate', 10)
        self.mfc1_publisher = self.create_publisher(Float32, 'mfc1/set_flow_rate', 10)
        self.current_valve = None
        self.delay_time = 0.2

        self.get_logger().info("Olfactometer Controller with HTTP interface initialized.")

    def trigger_stimulus(self, valve, ratio, duration, total_flow):
        if valve < 1 or valve > 16:
            self.get_logger().error("Invalid valve number.")
            return {"status": "error", "message": "Invalid valve number"}
        if not (0.0 <= ratio <= 1.0):
            self.get_logger().error("Invalid ratio.")
            return {"status": "error", "message": "Invalid ratio"}
        if total_flow < 0.0 or total_flow > 20.0:
            self.get_logger().error("Invalid total flow rate.")
            return {"status": "error", "message": "Total flow must be between 0.0 and 20.0 LPM"}

        flow_mfc0 = total_flow * ratio
        flow_mfc1 = total_flow * (1 - ratio)

        self.open_valve(valve)
        if self.current_valve is not None and self.current_valve != valve:
            time.sleep(self.delay_time)
            self.close_valve(self.current_valve)
        self.current_valve = valve

        self.mfc0_publisher.publish(Float32(data=flow_mfc0))
        self.mfc1_publisher.publish(Float32(data=flow_mfc1))
        self.get_logger().info(f"Set MFC0 to {flow_mfc0} LPM, MFC1 to {flow_mfc1} LPM")

        threading.Thread(target=self._close_after_delay, args=(valve, duration), daemon=True).start()
        return {"status": "success"}


    def _close_after_delay(self, valve, delay):
        time.sleep(delay)
        if self.current_valve == valve:
            self.close_valve(valve)
            self.current_valve = None
            self.reset_mfcs()

    def close_valve(self, valve_number):
        msg = String()
        msg.data = f"{valve_number}:OFF"
        self.valve_publisher.publish(msg)
        self.get_logger().info(f"Closed valve {valve_number}")

    def open_valve(self, valve_number):
        msg = String()
        msg.data = f"{valve_number}:ON"
        self.valve_publisher.publish(msg)
        self.get_logger().info(f"Opened valve {valve_number}")

    def reset_mfcs(self):
        self.mfc0_publisher.publish(Float32(data=0.0))
        self.mfc1_publisher.publish(Float32(data=0.0))
        self.get_logger().info("Reset MFCs to 0 LPM")

# --- FastAPI App ---
app = FastAPI()
ros_node = None  # We'll assign the ROS node after init

@app.post("/stimulus")
def stimulus_endpoint(req: StimulusRequest):
    return ros_node.trigger_stimulus(req.valve, req.ratio, req.duration, req.total_flow)


# --- Main entry point ---
def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = OlfactometerController()

    # Run FastAPI server in a separate thread
    api_thread = threading.Thread(
        target=uvicorn.run,
        args=("olfacto_web_control.lorig_server:app",),
        kwargs={"host": "0.0.0.0", "port": 8000, "log_level": "info"},
        daemon=True
    )
    api_thread.start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        ros_node.get_logger().info("Shutting down.")
    finally:
        if ros_node.current_valve is not None:
            ros_node.close_valve(ros_node.current_valve)
        ros_node.reset_mfcs()
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
