import importlib
from threading import Thread

import blockly_runtime_program
from metaclasses import Singleton


class BlocklyService(metaclass=Singleton):

    __program_thread = None
    __should_terminate = False
    __is_running = False

    def start_program(self):
        self.__is_running = True
        self.__should_terminate = False

        if self.__program_thread is None or not self.__program_thread.is_alive():
            importlib.reload(blockly_runtime_program)
            
            # TODO: Add functions to args
            self.__program_thread = Thread(target=blockly_runtime_program.main, args=())
            self.__program_thread.start()

    def stop_program(self):
        self.__should_terminate = True
        self.__is_running = False

    # FIXME: remove this logic from here, it should be like util func or some class
    def set_active_program(self):
        pass

    # FIXME: remove this logic from here, it should be like util func or some class
    def get_active_program(self):
        pass

    # FIXME: remove this logic from here, it should be like util func or some class
    def get_active_block_id(self):
        pass

    def is_program_running(self):
        return self.__is_running

    def should_terminate(self):
        return self.__should_terminate
