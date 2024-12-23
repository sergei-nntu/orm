from flask import Blueprint


blockly = Blueprint('blockly', __name__)


@blockly.route('/programs', methods=['GET'])
def get_blockly_programs():
    return 'Get active / non active program list'


@blockly.route('/start', methods=['POST'])
def start_blockly_program():
    return 'Start selected program'


@blockly.route('/stop', methods=['POST'])
def stop_blockly_program():
    return 'Stop selected program'


@blockly.route('/save', methods=['POST'])
def save_blockly_program():
    return 'Save selected program'