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
        super().__init__('olfactometer_controller_vacuum')
        self.valve_publisher = self.create_publisher(String, 'control_valve', 10)
        self.mfc0_publisher = self.create_publisher(Float32, 'mfc0/set_flow_rate', 10)
        self.mfc1_publisher = self.create_publisher(Float32, 'mfc1/set_flow_rate', 10)
        self.current_valve = None
        self.delay_time = 0.2

        self.get_logger().info("Olfactometer Controller with HTTP interface initialized.")

    def trigger_stimulus(self, valve, ratio, duration, total_flow):
        if not (0 <= valve <= 16):
            self.get_logger().error("Invalid valve number. Must be 0 (no valve) or 1–16.")
            return {"status": "error", "message": "Invalid valve number"}
        if not (0.0 <= ratio <= 1.0):
            self.get_logger().error("Invalid ratio.")
            return {"status": "error", "message": "Invalid ratio"}
        if total_flow < 0.0 or total_flow > 20.0:
            self.get_logger().error("Invalid total flow rate.")
            return {"status": "error", "message": "Total flow must be between 0.0 and 20.0 LPM"}

        flow_mfc0 = total_flow * ratio
        flow_mfc1 = total_flow * (1 - ratio)
        flow_mfc2 = total_flow

        # Close previously open odor valve if different
        if self.current_valve is not None and self.current_valve != valve:
            self.close_valve(self.current_valve)
            time.sleep(self.delay_time)

        if valve != 0:
            # Odor stimulus
            self.switch_3way_valve(True)  # ON = route to stimulus line
            self.open_valve(valve)
            self.current_valve = valve
            self.get_logger().info(f"Stimulus ON: valve {valve}")
        else:
            # Clean control air
            self.switch_3way_valve(False)  # OFF = route to control line
            self.get_logger().info("Control phase (no odor valve)")
            self.current_valve = None

        # Always publish flows
        self.mfc0_publisher.publish(Float32(data=flow_mfc0))
        self.mfc1_publisher.publish(Float32(data=flow_mfc1))
        self.mfc2_publisher.publish(Float32(data=flow_mfc2))
        self.get_logger().info(f"MFC0={flow_mfc0} | MFC1={flow_mfc1} | MFC2={flow_mfc2}")

        threading.Thread(target=self._close_after_delay, args=(valve, duration), daemon=True).start()
        return {"status": "success"}

    def switch_3way_valve(self, state_on: bool):
        """ Control valve 17 (3-way valve) to switch odor vs control air path """
        msg = String()
        msg.data = "17:ON" if state_on else "17:OFF"
        self.valve_publisher.publish(msg)
        self.get_logger().info(f"Switched 3-way valve to {'ODOR' if state_on else 'CONTROL'}")


    def _close_after_delay(self, valve, delay):
        time.sleep(delay)
        if self.current_valve == valve and valve != 0:
            self.close_valve(valve)
            self.get_logger().info(f"Stimulus valve {valve} closed after {delay}s")
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
        self.mfc2_publisher.publish(Float32(data=0.0))
        self.get_logger().info("Reset all MFCs to 0 LPM")


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
