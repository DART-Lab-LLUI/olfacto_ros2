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

        # --- Preload settings ---
        self.preload_delay = 2.0
        self.control_preload_delay = 2.0
        self.odr_boost_factor = 2.0
        self.ctrl_boost_factor = 1.2
        self.delay_time = 0.2

        self.get_logger().info("Vacuum olfactometer controller with preload boost initialized.")

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

        if valve != 0:
            # --- ODOR PRELOAD WITH BOOST ---
            boosted_total = total_flow * self.odr_boost_factor
            flow_mfc0_boost = boosted_total * ratio
            flow_mfc1_boost = boosted_total * (1 - ratio)

            self.open_valve(valve)
            self.current_valve = valve
            self.mfc0_publisher.publish(Float32(data=flow_mfc0_boost))
            self.mfc1_publisher.publish(Float32(data=flow_mfc1_boost))
            self.mfc2_publisher.publish(Float32(data=total_flow))

            self.get_logger().info(f"Preloading odor from valve {valve} for {self.preload_delay}s "
                                   f"with boosted flow (MFC0={flow_mfc0_boost}, MFC1={flow_mfc1_boost})")

            threading.Thread(
                target=self._delayed_switch_and_stimulus,
                args=(valve, ratio, duration, total_flow, self.preload_delay),
                daemon=True
            ).start()
        else:
            # --- CONTROL PRELOAD WITH BOOST ---
            flow_mfc0 = total_flow * ratio
            flow_mfc1 = total_flow * (1 - ratio)
            boosted_total = total_flow * self.ctrl_boost_factor

            self.mfc0_publisher.publish(Float32(data=flow_mfc0))
            self.mfc1_publisher.publish(Float32(data=flow_mfc1))  # flush odor line
            self.mfc2_publisher.publish(Float32(data=boosted_total))  # control line preload

            self.get_logger().info(f"Preloading control line for {self.control_preload_delay}s "
                                   f"with boosted flow {boosted_total} LPM")

            threading.Thread(
                target=self._delayed_control_switch,
                args=(duration, total_flow, self.control_preload_delay),
                daemon=True
            ).start()

        return {"status": "success"}

    def _delayed_switch_and_stimulus(self, valve, ratio, duration, total_flow, preload_delay):
        time.sleep(preload_delay)
        self.switch_3way_valve(True)
        self.get_logger().info("Switched 3-way valve to ODOR after preload")

        # Restore target flows
        flow_mfc0 = total_flow * ratio
        flow_mfc1 = total_flow * (1 - ratio)

        self.mfc0_publisher.publish(Float32(data=flow_mfc0))
        self.mfc1_publisher.publish(Float32(data=flow_mfc1))
        self.mfc2_publisher.publish(Float32(data=total_flow))

        self.get_logger().info(f"Set target flows: MFC0={flow_mfc0}, MFC1={flow_mfc1}, MFC2={total_flow}")

        time.sleep(duration)

        if self.current_valve == valve:
            self.close_valve(valve)
            self.current_valve = None

    def _delayed_control_switch(self, duration, total_flow, preload_delay):
        time.sleep(preload_delay)
        self.switch_3way_valve(False)
        self.get_logger().info("Switched 3-way valve to CONTROL after preload")

        # Restore target flows
        self.mfc1_publisher.publish(Float32(data=total_flow))  # odor line flush
        self.mfc2_publisher.publish(Float32(data=total_flow))  # control line

        self.get_logger().info(f"Set target control flows: MFC1={total_flow}, MFC2={total_flow}")

        time.sleep(duration)

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
        self.mfc1_publisher.publish(Float32(data=4.0))
        self.mfc2_publisher.publish(Float32(data=4.0))
        self.get_logger().info("Reset MFC0 to 0 LPM and MFC1,MFC2 to 4 LPM")


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
