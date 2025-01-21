import importlib
import textwrap
from threading import Thread

from metaclasses import Singleton


class BlocklyService(metaclass=Singleton):

    __program_thread = None

    __active_block_id = None
    __active_program_structure = None

    __is_running = False
    __should_terminate = False

    def start_program(self):
        from data import blockly_runtime_program

        self.__is_running = True
        self.__should_terminate = False

        if self.__program_thread is None or not self.__program_thread.is_alive():
            importlib.reload(blockly_runtime_program)

            # TODO: Add functions to args
            self.__program_thread = Thread(target=blockly_runtime_program.program, args=())
            self.__program_thread.start()

    def stop_program(self):
        self.__should_terminate = True
        self.__is_running = False

    def get_active_block_id(self):
        pass

    def is_program_running(self):
        return self.__is_running

    def should_terminate(self):
        return self.__should_terminate
    
    def set_active_program_structure(self, structure):
        self.__active_program_structure = structure

    # FIXME: active -> current
    def get_active_program_structure(self):
        return self.__active_program_structure

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

        with open('data/blockly_runtime_program.py', 'w') as file:
            file.write(program_template)
