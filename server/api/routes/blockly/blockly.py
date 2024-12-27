from flask import Blueprint, jsonify, g


blockly = Blueprint('blockly', __name__)


@blockly.route('/programs', methods=['GET'])
def get_blockly_programs():
    query = "SELECT id, program_name FROM programs"
    try:
        programs = g.db.execute(query)
        return jsonify({'programs': programs}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@blockly.route('/start', methods=['POST'])
def start_blockly_program():
    return 'Start selected program'


@blockly.route('/stop', methods=['POST'])
def stop_blockly_program():
    return 'Stop selected program'


@blockly.route('/save', methods=['POST'])
def save_blockly_program():
    return 'Save selected program'