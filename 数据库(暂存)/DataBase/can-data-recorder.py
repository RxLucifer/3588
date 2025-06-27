import os
import pymysql
from datetime import datetime
from threading import Thread, Event
import can

class CANDataRecorder:
    def __init__(self, can_interface='can0'):
        self.is_recording = Event()
        self.can_interface = can_interface
        self.current_file = None

    def run(self):
        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = self.can_interface
        bus = can.interface.Bus()

        while self.is_recording.is_set():
            message = bus.recv(1.0)
            if message and message.arbitration_id == 0x410:
                can_data = {
                    'opd_n_engine_rpm': message.data[0],
                    'opd_vel_veh_speed': message.data[1],
                    'opd_radio_steer': int.from_bytes(message.data[2:4], byteorder='big'),
                    'opd_tcu_app': message.data[4],
                    'opd_tcu_brake': message.data[5],
                    'opd_tcu_gear': message.data[6],
                    'opd_vehicle_state': message.data[7]
                }
                print(f"Recorded data: {can_data}")

                if self.current_file:
                    with open(self.current_file, 'a') as f:
                        data_line = f"{can_data['opd_n_engine_rpm']},{can_data['opd_vel_veh_speed']}," \
                                    f"{can_data['opd_radio_steer']},{can_data['opd_tcu_app']}," \
                                    f"{can_data['opd_tcu_brake']},{can_data['opd_tcu_gear']}," \
                                    f"{can_data['opd_vehicle_state']}\n"
                        f.write(data_line)

    def start_recording(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"/home/ztl/DataBase/record/can_{timestamp}.txt"
        self.current_file = filename
        with open(self.current_file, 'w') as f:
            f.write("engine_rpm,vehicle_speed,steer_angle,throttle_position,brake_pressure,transmission_gear,vehicle_state\n")
        self.is_recording.set()
        Thread(target=self.run).start()
        print(f"Recording to file: {self.current_file}")

    def stop_recording(self):
        self.is_recording.clear()
        print("Recording stopped.")
        return self.current_file

def list_txt_files(directory):
    return [f for f in os.listdir(directory) if f.endswith('.txt')]

def insert_data_to_database(filepath):
    db_config = {
        'host': 'localhost',
        'user': 'root',
        'password': '1234',
        'database': 'bus'
    }
    connection = pymysql.connect(**db_config)
    try:
        with connection.cursor() as cursor:
            with open(filepath, 'r') as f:
                header = f.readline()  # Skip the header line
                for line in f:
                    data = line.strip().split(',')
                    sql = """
                    INSERT INTO vehicle_data (engine_rpm, vehicle_speed, steer_angle, throttle_position, brake_pressure, transmission_gear, vehicle_state) 
                    VALUES (%s, %s, %s, %s, %s, %s, %s)
                    """
                    cursor.execute(sql, data)
            connection.commit()
        print("Data inserted successfully.")
    except Exception as e:
        print(f"Database insertion error: {e}")
    finally:
        connection.close()

def main():
    recorder = CANDataRecorder()
    while True:
        choice = input("Enter your choice (R: Record, I: Insert, Q: Quit): ").lower()
        if choice == 'r':
            recorder.start_recording()
            while True:
                stop_choice = input("Press 'S' to stop recording: ").lower()
                if stop_choice == 's':
                    recorder.stop_recording()
                    break
        elif choice == 'i':
            files = list_txt_files("/home/ztl/DataBase/record/")
            if not files:
                print("No data files found.")
                continue
            print("Available files:")
            for idx, file in enumerate(files):
                print(f"{idx + 1}: {file}")
            file_choice = int(input("Select a file to insert (number): "))
            if 1 <= file_choice <= len(files):
                filepath = os.path.join("/home/ztl/DataBase/record/", files[file_choice - 1])
                insert_data_to_database(filepath)
            else:
                print("Invalid choice.")
        elif choice == 'q':
            print("Exiting program.")
            break
        else:
            print("Invalid input. Please try again.")

if __name__ == '__main__':
    main()
