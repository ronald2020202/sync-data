#!/usr/bin/env python3
"""
Raspberry Pi WiFi Client for Arduino R4 WiFi Motor Control
Communicates with Arduino over WiFi for PID parameter tuning and control
"""

import socket
import time
import threading
import json
from datetime import datetime
from typing import Dict, Optional, Tuple
import re

class ArduinoPIDController:
    def __init__(self, arduino_ip: str, arduino_port: int = 80, timeout: float = 5.0):
        self.arduino_ip = arduino_ip
        self.arduino_port = arduino_port
        self.timeout = timeout
        self.socket = None
        self.connected = False
        self.status_data = {}
        self.status_callback = None
        self.running = True
        
        # Start status monitoring thread
        self.status_thread = threading.Thread(target=self._status_monitor, daemon=True)
        self.status_thread.start()
    
    def connect(self) -> bool:
        """Connect to Arduino over WiFi"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.arduino_ip, self.arduino_port))
            self.connected = True
            print(f"Connected to Arduino at {self.arduino_ip}:{self.arduino_port}")
            
            # Wait for connection message
            response = self._read_response()
            print(f"Arduino: {response}")
            
            return True
            
        except Exception as e:
            print(f"Failed to connect to Arduino: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        self.running = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.connected = False
        print("Disconnected from Arduino")
    
    def send_command(self, command: str) -> str:
        """Send command to Arduino and return response"""
        if not self.connected:
            return "ERROR Not connected"
        
        try:
            self.socket.send((command + '\n').encode())
            response = self._read_response()
            return response
        except Exception as e:
            print(f"Error sending command: {e}")
            self.connected = False
            return f"ERROR {e}"
    
    def _read_response(self) -> str:
        """Read response from Arduino"""
        try:
            response = self.socket.recv(1024).decode().strip()
            return response
        except Exception as e:
            return f"ERROR {e}"
    
    def _status_monitor(self):
        """Background thread to monitor status updates"""
        while self.running and self.connected:
            try:
                if self.socket:
                    # Set socket to non-blocking for status monitoring
                    self.socket.settimeout(0.1)
                    try:
                        data = self.socket.recv(1024).decode().strip()
                        if data.startswith("STATUS "):
                            self._parse_status(data[7:])  # Remove "STATUS " prefix
                        elif self.status_callback:
                            self.status_callback(data)
                    except socket.timeout:
                        pass
                    except Exception as e:
                        if self.connected:
                            print(f"Status monitor error: {e}")
                        break
            except:
                break
            time.sleep(0.05)
    
    def _parse_status(self, status_string: str):
        """Parse status string into dictionary"""
        try:
            # Parse comma-separated key:value pairs
            pairs = status_string.split(',')
            status_dict = {}
            
            for pair in pairs:
                if ':' in pair:
                    key, value = pair.split(':', 1)
                    # Try to convert to appropriate type
                    if value.lower() == 'true':
                        value = True
                    elif value.lower() == 'false':
                        value = False
                    else:
                        try:
                            # Try int first, then float
                            if '.' in value:
                                value = float(value)
                            else:
                                value = int(value)
                        except ValueError:
                            # Keep as string
                            pass
                    
                    status_dict[key] = value
            
            self.status_data = status_dict
            
            if self.status_callback:
                self.status_callback(status_dict)
                
        except Exception as e:
            print(f"Error parsing status: {e}")
    
    # Convenience methods for common operations
    def set_pid_parameters(self, kp: float, ki: float, kd: float) -> str:
        """Set PID parameters"""
        return self.send_command(f"PID {kp} {ki} {kd}")
    
    def set_target(self, target_ticks: int) -> str:
        """Set target position"""
        return self.send_command(f"TARGET {target_ticks}")
    
    def enable_pid(self) -> str:
        """Enable PID control"""
        return self.send_command("PID_ON")
    
    def disable_pid(self) -> str:
        """Disable PID control"""
        return self.send_command("PID_OFF")
    
    def stop_motor(self) -> str:
        """Stop motor"""
        return self.send_command("STOP")
    
    def zero_encoder(self) -> str:
        """Zero the encoder"""
        return self.send_command("ZERO")
    
    def get_status(self) -> Dict:
        """Get current status"""
        self.send_command("STATUS")
        time.sleep(0.1)  # Give time for response
        return self.status_data.copy()
    
    def motor_forward(self, speed: int = 100) -> str:
        """Move motor forward"""
        if speed != 100:
            return self.send_command(f"SPEED {speed}")
        else:
            return self.send_command("FORWARD")
    
    def motor_reverse(self, speed: int = 100) -> str:
        """Move motor reverse"""
        return self.send_command("REVERSE")


class PIDTuningSession:
    """Class to handle PID tuning sessions with data logging"""
    
    def __init__(self, controller: ArduinoPIDController):
        self.controller = controller
        self.session_data = []
        self.session_start = datetime.now()
    
    def run_test(self, kp: float, ki: float, kd: float, target: int, duration: float = 10.0) -> Dict:
        """Run a PID test and collect performance data"""
        print(f"\nTesting PID: Kp={kp}, Ki={ki}, Kd={kd}, Target={target}")
        
        # Reset system
        self.controller.disable_pid()
        self.controller.zero_encoder()
        time.sleep(0.5)
        
        # Set PID parameters
        response = self.controller.set_pid_parameters(kp, ki, kd)
        print(f"PID set: {response}")
        
        # Set target and enable PID
        self.controller.set_target(target)
        self.controller.enable_pid()
        
        # Collect data during test
        start_time = time.time()
        test_data = []
        
        print("Collecting data...")
        while time.time() - start_time < duration:
            status = self.controller.get_status()
            timestamp = time.time() - start_time
            
            test_data.append({
                'timestamp': timestamp,
                'ticks': status.get('ticks', 0),
                'target': status.get('target', 0),
                'error': status.get('error', 0),
                'stable_count': status.get('stable_count', 0)
            })
            
            # Print progress
            error = status.get('error', 0)
            stable = status.get('stable_count', 0)
            print(f"Time: {timestamp:.1f}s, Error: {error}, Stable: {stable}/50", end='\r')
            
            time.sleep(0.1)
        
        print()  # New line
        
        # Stop PID
        self.controller.disable_pid()
        
        # Analyze results
        results = self._analyze_test_data(test_data, kp, ki, kd, target)
        self.session_data.append(results)
        
        return results
    
    def _analyze_test_data(self, test_data: list, kp: float, ki: float, kd: float, target: int) -> Dict:
        """Analyze test data and return performance metrics"""
        if not test_data:
            return {}
        
        errors = [abs(d['error']) for d in test_data]
        stable_counts = [d['stable_count'] for d in test_data]
        
        # Calculate metrics
        mean_error = sum(errors) / len(errors)
        max_error = max(errors)
        final_error = errors[-1]
        
        # Check if it stabilized
        stabilized = any(sc >= 50 for sc in stable_counts)
        if stabilized:
            # Find first stabilization time
            stabilization_time = next(
                (d['timestamp'] for d in test_data if d['stable_count'] >= 50),
                float('inf')
            )
        else:
            stabilization_time = float('inf')
        
        # Calculate settling time (time to get within 5% of target)
        settling_threshold = abs(target) * 0.05
        settling_time = float('inf')
        for data_point in test_data:
            if abs(data_point['error']) <= settling_threshold:
                settling_time = data_point['timestamp']
                break
        
        results = {
            'timestamp': datetime.now().isoformat(),
            'kp': kp,
            'ki': ki,
            'kd': kd,
            'target': target,
            'mean_error': mean_error,
            'max_error': max_error,
            'final_error': final_error,
            'stabilized': stabilized,
            'stabilization_time': stabilization_time,
            'settling_time': settling_time,
            'test_duration': test_data[-1]['timestamp'],
            'raw_data': test_data
        }
        
        return results
    
    def save_session(self, filename: Optional[str] = None):
        """Save tuning session data to file"""
        if filename is None:
            timestamp = self.session_start.strftime("%Y%m%d_%H%M%S")
            filename = f"pid_tuning_session_{timestamp}.json"
        
        session_summary = {
            'session_start': self.session_start.isoformat(),
            'session_end': datetime.now().isoformat(),
            'total_tests': len(self.session_data),
            'tests': self.session_data
        }
        
        with open(filename, 'w') as f:
            json.dump(session_summary, f, indent=2)
        
        print(f"Session data saved to {filename}")
        return filename


def interactive_tuning_session():
    """Interactive PID tuning session"""
    print("=== Interactive PID Tuning Session ===")
    
    # Get Arduino IP address
    arduino_ip = input("Enter Arduino IP address: ").strip()
    if not arduino_ip:
        print("No IP address provided. Exiting.")
        return
    
    # Connect to Arduino
    controller = ArduinoPIDController(arduino_ip)
    
    if not controller.connect():
        print("Failed to connect to Arduino")
        return
    
    try:
        # Create tuning session
        session = PIDTuningSession(controller)
        
        print("\nCommands:")
        print("test kp ki kd target [duration] - Run PID test")
        print("status - Show current status")
        print("zero - Zero encoder")
        print("stop - Stop motor")
        print("save - Save session data")
        print("quit - Exit")
        
        while True:
            command = input("\n> ").strip().lower()
            
            if command == "quit":
                break
            elif command == "status":
                status = controller.get_status()
                print("Current Status:")
                for key, value in status.items():
                    print(f"  {key}: {value}")
            elif command == "zero":
                response = controller.zero_encoder()
                print(response)
            elif command == "stop":
                response = controller.stop_motor()
                print(response)
            elif command == "save":
                session.save_session()
            elif command.startswith("test "):
                # Parse test command: test kp ki kd target [duration]
                parts = command.split()
                if len(parts) >= 5:
                    try:
                        kp = float(parts[1])
                        ki = float(parts[2])
                        kd = float(parts[3])
                        target = int(parts[4])
                        duration = float(parts[5]) if len(parts) > 5 else 10.0
                        
                        results = session.run_test(kp, ki, kd, target, duration)
                        
                        print("\nTest Results:")
                        print(f"  Mean Error: {results['mean_error']:.2f}")
                        print(f"  Max Error: {results['max_error']:.2f}")
                        print(f"  Final Error: {results['final_error']:.2f}")
                        print(f"  Stabilized: {results['stabilized']}")
                        if results['stabilization_time'] != float('inf'):
                            print(f"  Stabilization Time: {results['stabilization_time']:.2f}s")
                        if results['settling_time'] != float('inf'):
                            print(f"  Settling Time: {results['settling_time']:.2f}s")
                        
                    except ValueError as e:
                        print(f"Invalid parameters: {e}")
                else:
                    print("Usage: test kp ki kd target [duration]")
            else:
                print("Unknown command. Type 'quit' to exit.")
    
    except KeyboardInterrupt:
        print("\nSession interrupted")
    finally:
        controller.disconnect()


def discover_arduino():
    """Try to discover Arduino on the local network"""
    print("Scanning for Arduino on local network...")
    
    # Get local network range (simple approach)
    import subprocess
    try:
        # Get local IP
        result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
        local_ip = result.stdout.strip().split()[0]
        
        # Extract network prefix (e.g., 192.168.1.)
        network_prefix = '.'.join(local_ip.split('.')[:-1]) + '.'
        
        print(f"Scanning network: {network_prefix}1-254")
        
        # Quick scan of common IP addresses
        potential_ips = []
        for i in range(1, 255):
            ip = f"{network_prefix}{i}"
            try:
                # Quick connection test
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(0.5)
                result = sock.connect_ex((ip, 80))
                sock.close()
                
                if result == 0:
                    potential_ips.append(ip)
                    print(f"Found device at {ip}")
            except:
                pass
        
        return potential_ips
    
    except Exception as e:
        print(f"Network scan failed: {e}")
        return []


def rl_training_interface(controller: ArduinoPIDController):
    """Interface for RL training - provides methods for automated PID testing"""
    
    class RLInterface:
        def __init__(self, ctrl):
            self.controller = ctrl
            self.episode_count = 0
        
        def reset_episode(self) -> Dict:
            """Reset for new episode and return initial state"""
            self.controller.disable_pid()
            self.controller.zero_encoder()
            time.sleep(0.5)
            
            # Get initial state
            status = self.controller.get_status()
            self.episode_count += 1
            
            print(f"Episode {self.episode_count} started")
            return status
        
        def step(self, pid_params: Tuple[float, float, float], target: int, duration: float = 5.0) -> Tuple[Dict, float, bool]:
            """
            Run one RL step: apply PID parameters and return state, reward, done
            
            Returns:
                state: Current system state
                reward: Performance reward
                done: Whether episode is complete
            """
            kp, ki, kd = pid_params
            
            # Set PID parameters
            self.controller.set_pid_parameters(kp, ki, kd)
            self.controller.set_target(target)
            self.controller.enable_pid()
            
            # Collect data for specified duration
            start_time = time.time()
            errors = []
            stable_counts = []
            
            while time.time() - start_time < duration:
                status = self.controller.get_status()
                errors.append(abs(status.get('error', 0)))
                stable_counts.append(status.get('stable_count', 0))
                time.sleep(0.1)
            
            # Calculate reward based on performance
            mean_error = sum(errors) / len(errors) if errors else float('inf')
            final_error = errors[-1] if errors else float('inf')
            max_stable = max(stable_counts) if stable_counts else 0
            stabilized = max_stable >= 50
            
            # Reward function - customize based on your goals
            reward = 0.0
            
            # Reward for low error
            reward += max(0, 100 - mean_error)
            
            # Bonus for stabilization
            if stabilized:
                reward += 50
            
            # Bonus for fast stabilization
            if stabilized:
                stabilization_time = next(
                    (i * 0.1 for i, sc in enumerate(stable_counts) if sc >= 50),
                    duration
                )
                reward += max(0, 50 - stabilization_time * 10)
            
            # Penalty for high final error
            reward -= final_error * 2
            
            # Get final state
            final_status = self.controller.get_status()
            
            # Episode is done if stabilized or duration elapsed
            done = stabilized or (time.time() - start_time >= duration)
            
            return final_status, reward, done
    
    return RLInterface(controller)


def manual_tuning_wizard():
    """Guided manual tuning process"""
    print("\n=== PID Tuning Wizard ===")
    print("This wizard will guide you through manual PID tuning")
    
    # Auto-discover or manual IP entry
    print("\nStep 1: Connect to Arduino")
    discovered_ips = discover_arduino()
    
    if discovered_ips:
        print(f"Found {len(discovered_ips)} potential Arduino(s):")
        for i, ip in enumerate(discovered_ips):
            print(f"  {i+1}. {ip}")
        
        choice = input("Select Arduino (number) or enter custom IP: ").strip()
        
        if choice.isdigit() and 1 <= int(choice) <= len(discovered_ips):
            arduino_ip = discovered_ips[int(choice) - 1]
        else:
            arduino_ip = choice
    else:
        arduino_ip = input("Enter Arduino IP address: ").strip()
    
    # Connect
    controller = ArduinoPIDController(arduino_ip)
    if not controller.connect():
        return
    
    try:
        print("\nStep 2: System Check")
        
        # Zero encoder
        print("Zeroing encoder...")
        controller.zero_encoder()
        time.sleep(1)
        
        # Check initial status
        status = controller.get_status()
        print(f"Initial position: {status.get('ticks', 0)} ticks")
        
        print("\nStep 3: Manual Movement Test")
        print("Testing manual motor control...")
        
        input("Press Enter to test forward movement...")
        controller.motor_forward(50)
        time.sleep(2)
        controller.stop_motor()
        
        status = controller.get_status()
        print(f"Position after forward: {status.get('ticks', 0)} ticks")
        
        input("Press Enter to test reverse movement...")
        controller.motor_reverse(50)
        time.sleep(2)
        controller.stop_motor()
        
        status = controller.get_status()
        print(f"Position after reverse: {status.get('ticks', 0)} ticks")
        
        controller.zero_encoder()
        
        print("\nStep 4: PID Tuning")
        print("Now we'll test different PID parameters")
        
        # Create tuning session
        session = PIDTuningSession(controller)
        
        # Suggested tuning sequence
        suggested_tests = [
            (1.0, 0.0, 0.0, 1000),   # P only
            (2.0, 0.0, 0.0, 1000),   # Higher P
            (2.0, 0.0, 0.1, 1000),   # Add some D
            (2.0, 0.1, 0.1, 1000),   # Add some I
        ]
        
        print("Suggested test sequence:")
        for i, (kp, ki, kd, target) in enumerate(suggested_tests):
            print(f"  {i+1}. Kp={kp}, Ki={ki}, Kd={kd}, Target={target}")
        
        auto_mode = input("Run suggested tests automatically? (y/n): ").lower() == 'y'
        
        if auto_mode:
            print("Running automatic test sequence...")
            for kp, ki, kd, target in suggested_tests:
                results = session.run_test(kp, ki, kd, target, duration=8.0)
                
                print(f"Results: Mean Error={results['mean_error']:.1f}, "
                      f"Stabilized={results['stabilized']}, "
                      f"Final Error={results['final_error']:.1f}")
                
                time.sleep(2)  # Brief pause between tests
        else:
            # Manual mode
            while True:
                command = input("\nEnter: 'test kp ki kd target' or 'done': ").strip()
                
                if command.lower() == 'done':
                    break
                
                if command.startswith('test '):
                    parts = command.split()
                    if len(parts) == 5:
                        try:
                            kp = float(parts[1])
                            ki = float(parts[2])
                            kd = float(parts[3])
                            target = int(parts[4])
                            
                            results = session.run_test(kp, ki, kd, target)
                            
                            print(f"Results: Mean Error={results['mean_error']:.1f}, "
                                  f"Stabilized={results['stabilized']}, "
                                  f"Final Error={results['final_error']:.1f}")
                        except ValueError:
                            print("Invalid parameters")
                    else:
                        print("Usage: test kp ki kd target")
        
        # Save results
        filename = session.save_session()
        print(f"\nTuning session complete!")
        print(f"Data saved to: {filename}")
        
        # Show best results
        if session.session_data:
            best_test = min(session.session_data, key=lambda x: x['mean_error'])
            print(f"\nBest parameters found:")
            print(f"  Kp={best_test['kp']}, Ki={best_test['ki']}, Kd={best_test['kd']}")
            print(f"  Mean Error: {best_test['mean_error']:.2f}")
            print(f"  Stabilized: {best_test['stabilized']}")
    
    except KeyboardInterrupt:
        print("\nTuning interrupted")
    finally:
        controller.disconnect()


def simple_rl_training_demo():
    """Simplified RL training demonstration"""
    print("\n=== Simple RL Training Demo ===")
    
    arduino_ip = input("Enter Arduino IP address: ").strip()
    controller = ArduinoPIDController(arduino_ip)
    
    if not controller.connect():
        return
    
    try:
        rl_interface = rl_training_interface(controller)
        
        print("Running simplified RL training...")
        print("This will test random PID parameters and learn which ones work best")
        
        # Simple random search (not true RL, but demonstrates the concept)
        best_reward = -float('inf')
        best_params = None
        
        for episode in range(10):  # Just 10 episodes for demo
            # Generate random PID parameters
            kp = np.random.uniform(0.5, 10.0)
            ki = np.random.uniform(0.0, 1.0)
            kd = np.random.uniform(0.0, 2.0)
            
            target = 2000  # Fixed target for consistency
            
            print(f"\nEpisode {episode + 1}: Testing Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}")
            
            # Reset for new episode
            rl_interface.reset_episode()
            
            # Run test
            state, reward, done = rl_interface.step((kp, ki, kd), target, duration=6.0)
            
            print(f"Reward: {reward:.2f}")
            
            if reward > best_reward:
                best_reward = reward
                best_params = (kp, ki, kd)
                print("*** New best parameters! ***")
            
            time.sleep(1)
        
        print(f"\nTraining complete!")
        print(f"Best parameters: Kp={best_params[0]:.3f}, Ki={best_params[1]:.3f}, Kd={best_params[2]:.3f}")
        print(f"Best reward: {best_reward:.2f}")
        
        # Test the best parameters
        print("\nTesting best parameters...")
        rl_interface.reset_episode()
        state, reward, done = rl_interface.step(best_params, 2000, duration=10.0)
        print(f"Final test reward: {reward:.2f}")
    
    except KeyboardInterrupt:
        print("\nTraining interrupted")
    finally:
        controller.disconnect()


if __name__ == "__main__":
    import numpy as np  # For random parameter generation
    
    print("=== Arduino R4 WiFi PID Controller Interface ===")
    print("1. Interactive tuning session")
    print("2. Simple RL training demo")
    print("3. Direct connection test")
    
    choice = input("Select option (1-3): ").strip()
    
    if choice == "1":
        manual_tuning_wizard()
    elif choice == "2":
        simple_rl_training_demo()
    elif choice == "3":
        arduino_ip = input("Enter Arduino IP: ").strip()
        controller = ArduinoPIDController(arduino_ip)
        if controller.connect():
            print("Connection successful!")
            print("Type commands like: 'FORWARD', 'STOP', 'STATUS', 'HELP'")
            try:
                while True:
                    cmd = input("> ").strip()
                    if cmd.lower() == 'quit':
                        break
                    response = controller.send_command(cmd)
                    print(f"Response: {response}")
            except KeyboardInterrupt:
                pass
            finally:
                controller.disconnect()
    else:
        print("Invalid choice")