from flask import request

from services import BlocklyService


def set_active_program():
    data = request.get_json()

    program = data.get('source')
    structure = data.get('structure')

    blockly = BlocklyService()

    blockly.write_program_to_file(program)
    blockly.set_active_program_structure(structure)

    return 'response'