import logging
from flask import Blueprint, jsonify, request
from types import SimpleNamespace
from dao import BlocklyDAO


blockly = Blueprint('blockly', __name__)
logger = logging.getLogger(__name__)

# blockly_thread = None
# program_running = None
# should_program_terminate = False


@blockly.get('/programs')
def get_programs():
    try:
        blockly_dao = BlocklyDAO()
        programs = blockly_dao.get_programs()
        logger.debug(f"Programs fetched: {programs}")
        return jsonify({'programs': programs}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@blockly.route('/start', methods=['POST'])
def start_program():
    data = request.get_json()
    id = data.get('id')

    dao = BlocklyDAO()
    return dao.get_program(id)

    # Algorithm
    # 1) Validate the input data
    # 2) Get the program from the data base by id
    # 3) Launch the program in the tread
    return 'Start selected program'

@blockly.route('/stop', methods=['POST'])
def stop_program():
    return 'Stop selected program'

@blockly.post('')
def save_program():
    data = request.get_json()

    program_data = {
        'name': data.get('program_name'),
        'description': data.get('program_description'),
        'structure': data.get('program_structure'),
        'is_running': data.get('is_running'),
    }

    program = SimpleNamespace(**program_data)

    dao = BlocklyDAO()
    dao.add_program(program)

    return 'Save selected program\n'

@blockly.patch('')
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

@blockly.delete('')
def delete_program():
    id = request.args.get('id')

    dao = BlocklyDAO()
    dao.delete_program(id)

    return f'Program with id = {id} was deleted\n'