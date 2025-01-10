import logging
from flask import Blueprint, jsonify, g


blockly = Blueprint('blockly', __name__)
logger = logging.getLogger(__name__)

blockly_thread = None
program_running = None
should_program_terminate = False


@blockly.route('/programs', methods=['GET'])
def get_blockly_programs():
    query = "SELECT id, program_name FROM programs"
    try:
        logger.info("Fetching Blockly programs from the database.")
        programs = g.db.execute(query)
        logger.debug(f"Programs fetched: {programs}")
        return jsonify({'programs': programs}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@blockly.route('/start', methods=['POST'])
def start_blockly_program():
    # Algorithm
    # 1) Validate the input data
    # 2) Get the program from the data base by id
    # 3) Launch the program in the tread
    return 'Start selected program'


@blockly.route('/stop', methods=['POST'])
def stop_blockly_program():
    return 'Stop selected program'


@blockly.route('/save', methods=['POST'])
def save_blockly_program():
    return 'Save selected program'