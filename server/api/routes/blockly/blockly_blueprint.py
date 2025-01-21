from flask import Blueprint

from .get_program import get_program
from .get_programs import get_programs
from .save_program import save_program
from .stop_program import stop_program
from .start_program import start_program
from .update_program import update_program
from .delete_program import delete_program
from .get_current_program import get_current_program

blockly = Blueprint('blockly', __name__)

blockly.add_url_rule('/blockly/<int:id>', view_func=get_program, methods=['GET'])

blockly.add_url_rule('/blockly', view_func=get_programs, methods=['GET'])
blockly.add_url_rule('/blockly', view_func=save_program, methods=['POST'])
blockly.add_url_rule('/blockly', view_func=update_program, methods=['PATCH'])
blockly.add_url_rule('/blockly', view_func=delete_program, methods=['DELETE'])

blockly.add_url_rule('/blockly/stop', view_func=stop_program, methods=['POST']) 
blockly.add_url_rule('/blockly/start', view_func=start_program, methods=['POST'])
blockly.add_url_rule('/blockly/current', view_func=get_current_program, methods=['GET'])
