import os
import time
import textwrap
import importlib
from threading import Thread

from metaclasses import Singleton


class BlocklyService(metaclass=Singleton):

    __program_thread = None

    __active_block_id = None
    __current_program_structure = None

    __is_running = False
    __should_terminate = False

    def start_program(self):
        from data import blockly_runtime_program

        self.__is_running = True
        self.__should_terminate = False

        if self.__program_thread is None or not self.__program_thread.is_alive():
            importlib.reload(blockly_runtime_program)

            self.__program_thread = Thread(target=blockly_runtime_program.program, args=(self.should_terminate, self.set_active_block_id, self.publish_grip_state, self.orm_blockly_delay, self.orm_blockly_set_position, self.orm_blockly_set_gripper_state))
            self.__program_thread.start()
    
    def stop_program(self):
        self.__should_terminate = True
        self.__is_running = False
    
    def publish_grip_state(self, state):
        print('publish_grip_state', state)
        time.sleep(1)

    def orm_blockly_delay(self, ms):
        time.sleep(ms / 1000)

    def orm_blockly_set_position(self, x, y, z, roll, pitch, yaw):
        print('orm_blockly_set_position')

    def orm_blockly_set_gripper_state(self, value):
        print('orm_blockly_set_gripper_state')

    def get_active_block_id(self):
        pass

    def is_program_running(self):
        return self.__is_running

    def should_terminate(self):
        return self.__should_terminate
    
    def set_active_block_id(self, value):
        self.__active_block_id = value

    def get_active_block_id(self):
        return self.__active_block_id

    def set_current_program_structure(self, value):
        self.__current_program_structure = value

    def get_current_program_structure(self):
        return self.__current_program_structure

    def save_program_structure(self, structure):
        self.set_current_program_structure(structure)

        data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data')

        if not os.path.exists(data_dir):
            raise FileNotFoundError(f"The data dir - {data_dir} doesn't exist")

        file_path = os.path.join(data_dir, 'blockly_program_structure.py')
        with open(file_path, 'w', encoding='utf-8') as file:
            file.write(str(structure))

    def get_program_structure(self):
        structure = self.get_current_program_structure()

        if not structure:
            data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data')

            if not os.path.exists(data_dir):
                raise FileNotFoundError(f"The data dir - {data_dir} doesn't exist")

            file_path = os.path.join(data_dir, 'blockly_program_structure.py')

            with open(file_path, 'r', encoding='utf-8') as file:
                structure = file.read()

        return structure

    def write_program_to_file(self, program_code):
        indented_code = textwrap.indent(program_code, '  ')
        
        program_template = (
            "def program("
            "should_terminate_function, "
            "set_active_block_id, "
            "publish_grip_state, "
            "orm_blockly_delay, "
            "orm_blockly_set_position, "
            "orm_blockly_set_gripper_state"
            "):\n"
            f"{indented_code}"
        )

        data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data')

        if not os.path.exists(data_dir):
            raise FileNotFoundError(f"The data dir - {data_dir} doesn't exist")

        file_path = os.path.join(data_dir, 'blockly_runtime_program.py')

        with open(file_path, 'w', encoding='utf-8') as file:
            file.write(program_template)
