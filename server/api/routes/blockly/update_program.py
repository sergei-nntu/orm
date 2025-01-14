from flask import request
from types import SimpleNamespace

from dao import BlocklyDAO


def update_program():
    data = request.get_json()
    id = request.args.get('id')

    program_data = {
        'name': data.get('program_name'),
        'description': data.get('program_description'),
        'structure': data.get('program_structure'),
        'is_running': data.get('is_running'),
    }

    program = SimpleNamespace(**program_data)

    dao = BlocklyDAO()
    dao.update_program(program, id)

    return 'program'