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

# --- FastAPI app ---
app = FastAPI()

# --- ROS Node Class ---
class OlfactometerController(Node):
    def __init__(self):
        super().__init__('olfactometer_controller_vacuum')
        self.valve_publisher = self.create_publisher(String, 'control_valve', 10)
        self.mfc0_publisher = self.create_publisher(Float32, 'mfc0/set_flow_rate', 10)
        self.mfc1_publisher = self.create_publisher(Float32, 'mfc1/set_flow_rate', 10)
        self.mfc2_publisher = self.create_publisher(Float32, 'mfc2/set_flow_rate', 10)
        self.current_valve = None
        self.delay_time = 0.2
        self.preload_delay = 2.0  # seconds before switching 3-way valve

        self.get_logger().info("Vacuum olfactometer controller initialized.")

    def trigger_stimulus(self, valve, ratio, duration, total_flow):
        if not (0 <= valve <= 16):
            self.get_logger().error("Invalid valve number.")
            return {"status": "error", "message": "Valve must be between 0–16"}
        if not (0.0 <= ratio <= 1.0):
            self.get_logger().error("Invalid ratio.")
            return {"status": "error", "message": "Ratio must be between 0–1"}
        if not (0.0 <= total_flow <= 20.0):
            self.get_logger().error("Invalid flow.")
            return {"status": "error", "message": "Total flow must be between 0–20 LPM"}

        flow_mfc0 = total_flow * ratio
        flow_mfc1 = total_flow * (1 - ratio)
        flow_mfc2 = total_flow

        # Close previous valve if needed
        if self.current_valve and self.current_valve != valve:
            self.close_valve(self.current_valve)
            time.sleep(self.delay_time)

        if valve != 0:
            # --- ODOR PRELOAD PHASE ---
            self.open_valve(valve)
            self.current_valve = valve
            self.mfc0_publisher.publish(Float32(data=flow_mfc0))
            self.mfc1_publisher.publish(Float32(data=flow_mfc1))
            self.mfc2_publisher.publish(Float32(data=flow_mfc2))
            self.get_logger().info(f"Preloading odor from valve {valve} for {self.preload_delay}s")

            threading.Thread(
                target=self._delayed_switch_and_stimulus,
                args=(valve, duration, self.preload_delay),
                daemon=True
            ).start()
        else:
            # --- CONTROL AIR PHASE ---
            self.switch_3way_valve(False)  # route to control line
            self.mfc0_publisher.publish(Float32(data=0.0))
            self.mfc1_publisher.publish(Float32(data=flow_mfc1))  # flush odor line
            self.mfc2_publisher.publish(Float32(data=flow_mfc2))  # control air
            self.get_logger().info("Control phase active. Odor line flushed, control air on.")

            threading.Thread(
                target=self._control_phase_cleanup,
                args=(duration,),
                daemon=True
            ).start()

        return {"status": "success"}

    def _delayed_switch_and_stimulus(self, valve, duration, preload_delay):
        time.sleep(preload_delay)
        self.switch_3way_valve(True)
        self.get_logger().info(f"Switched 3-way valve to ODOR after preload")

        time.sleep(duration)
        if self.current_valve == valve:
            self.close_valve(valve)
            self.current_valve = None

    def _control_phase_cleanup(self, delay):
        time.sleep(delay)

    def switch_3way_valve(self, state_on: bool):
        msg = String()
        msg.data = "17:ON" if state_on else "17:OFF"
        self.valve_publisher.publish(msg)
        self.get_logger().info(f"3-way valve set to {'ODOR' if state_on else 'CONTROL'}")

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


# --- FastAPI route ---
@app.post("/stimulus")
def stimulus_endpoint(req: StimulusRequest):
    ros_node = app.state.ros_node
    return ros_node.trigger_stimulus(req.valve, req.ratio, req.duration, req.total_flow)

# --- Main entry point ---
def main(args=None):
    rclpy.init(args=args)
    ros_node = OlfactometerController()
    app.state.ros_node = ros_node

    api_thread = threading.Thread(
        target=uvicorn.run,
        args=("olfacto_web_control.vacuum_server:app",),
        kwargs={"host": "0.0.0.0", "port": 8000, "log_level": "info"},
        daemon=True
    )
    api_thread.start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        ros_node.get_logger().info("Shutting down.")
    finally:
        if ros_node.current_valve:
            ros_node.close_valve(ros_node.current_valve)
        ros_node.reset_mfcs()
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
